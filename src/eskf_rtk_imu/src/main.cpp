#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include "eskf_rtk_imu/imu_processor.hpp"
#include "eskf_rtk_imu/gps_processor.hpp"
#include "eskf_rtk_imu/base_type.hpp"
#include "eskf_rtk_imu/utils.hpp"
#include "Eigen/Geometry"
#include "std_msgs/msg/float64.hpp"

class ESKFFusionNode : public rclcpp::Node
{
public:
    ESKFFusionNode() : Node("eskf_rtk_imu_node"),
                       imu_processor_(nullptr),
                       gps_processor_(nullptr),
                       state_(std::make_shared<ErrorStateKalmanFilter::State>()),
                       last_imu_data_ptr(nullptr),
                       first_imu(true),
                       first_gps(true),
                       first_gps_yaw(true)
    {
        this->declare_parameter<double>("acc_noise", 0.01);
        this->declare_parameter<double>("gyro_noise", 0.01);
        this->declare_parameter<double>("acc_bias_noise", 0.0001);
        this->declare_parameter<double>("gyro_bias_noise", 0.0001);
        this->declare_parameter<double>("I_p_Gps_x", 0.0);
        this->declare_parameter<double>("I_p_Gps_y", 0.0);
        this->declare_parameter<double>("I_p_Gps_z", 0.0);

        this->get_parameter("acc_noise", acc_noise_);
        this->get_parameter("gyro_noise", gyro_noise_);
        this->get_parameter("acc_bias_noise", acc_bias_noise_);
        this->get_parameter("gyro_bias_noise", gyro_bias_noise_);
        double I_p_Gps_x_, I_p_Gps_y_, I_p_Gps_z_;
        this->get_parameter("I_p_Gps_x", I_p_Gps_x_);
        this->get_parameter("I_p_Gps_y", I_p_Gps_y_);
        this->get_parameter("I_p_Gps_z", I_p_Gps_z_);
        I_p_Gps_(0) = I_p_Gps_x_;
        I_p_Gps_(1) = I_p_Gps_y_;
        I_p_Gps_(2) = I_p_Gps_z_;
        gravity_(0) = 0.0;
        gravity_(1) = 0.0;
        gravity_(2) = -9.7944;
        state_->G_p_I = {0., 0., 0.};
        state_->cov.setIdentity();
        state_->cov.block<3, 3>(0, 0) *= 0.01;   // 位置：0.1 标准差
        state_->cov.block<3, 3>(3, 3) *= 0.04;   // 速度：~1 m/s
        state_->cov.block<3, 3>(6, 6) *= 0.001;  // 姿态角：~0.3 rad
        state_->cov.block<3, 3>(9, 9) *= 1e-4;   // 陀螺零偏
        state_->cov.block<3, 3>(12, 12) *= 1e-3; // 加速度零偏
        state_->cov.block<3, 3>(15, 15) *= 1e-6; // 重力

        imu_processor_ = std::make_shared<ErrorStateKalmanFilter::ImuProcessor>(acc_noise_, gyro_noise_, acc_bias_noise_, gyro_bias_noise_, gravity_);
        gps_processor_ = std::make_shared<ErrorStateKalmanFilter::GpsProcessor>(Eigen::Vector3d(I_p_Gps_x_, I_p_Gps_y_, I_p_Gps_z_));

        origin_rtk_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("handsfree/rtk/gnss",
                                                                                 rclcpp::SensorDataQoS(),
                                                                                 std::bind(&ESKFFusionNode::origin_rtk_callback, this, std::placeholders::_1));
        gnss_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>("handsfree/rtk/heading",
                                                                          rclcpp::SensorDataQoS(),
                                                                          std::bind(&ESKFFusionNode::origin_rtk_yaw_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data",
                                                                    rclcpp::SensorDataQoS(),
                                                                    std::bind(&ESKFFusionNode::origin_imu_callback, this, std::placeholders::_1));
        fusioned_gnss_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("rtk_imu_fusion_gnss",
                                                                                 rclcpp::SensorDataQoS());
        // fusioned_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("rtk_imu_fusion_odom",
        //                                                                         10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_rtk_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gnss_yaw_sub_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fusioned_gnss_pub_;
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fusioned_odom_pub_;

    double acc_noise_, gyro_noise_, acc_bias_noise_, gyro_bias_noise_;
    Eigen::Vector3d I_p_Gps_, gravity_;

    std::shared_ptr<ErrorStateKalmanFilter::ImuProcessor> imu_processor_;
    std::shared_ptr<ErrorStateKalmanFilter::GpsProcessor> gps_processor_;
    std::shared_ptr<ErrorStateKalmanFilter::State> state_;
    ErrorStateKalmanFilter::ImuDataPtr last_imu_data_ptr;
    bool first_imu, first_gps, first_gps_yaw;
    Eigen::Vector3d first_lla;

    void origin_rtk_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        ErrorStateKalmanFilter::GpsPositionDataPtr gps_data_ptr = std::make_shared<ErrorStateKalmanFilter::GpsPositionData>();
        gps_data_ptr->cov = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(msg->position_covariance.data());
        gps_data_ptr->lla.x() = msg->latitude;
        gps_data_ptr->lla.y() = msg->longitude;
        gps_data_ptr->lla.z() = msg->altitude;

        if (first_gps)
        {
            first_lla = gps_data_ptr->lla;
            ConvertLLAToENU(first_lla, gps_data_ptr->lla, &state_->G_p_I);
            first_gps = false;
        }
        else
        {
            gps_processor_->UpdateStateByGpsPosition(first_lla, gps_data_ptr, state_.get());
        }
    }
    void origin_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        ErrorStateKalmanFilter::ImuDataPtr imu_data_ptr = std::make_shared<ErrorStateKalmanFilter::ImuData>();
        rclcpp::Time imu_time = rclcpp::Time(msg->header.stamp);
        static double first_time = imu_time.seconds();
        imu_data_ptr->timestamp = imu_time.seconds();
        imu_data_ptr->acc(0) = msg->linear_acceleration.x;
        imu_data_ptr->acc(1) = msg->linear_acceleration.y;
        imu_data_ptr->acc(2) = msg->linear_acceleration.z;

        imu_data_ptr->gyro = {msg->angular_velocity.x,
                              msg->angular_velocity.y,
                              msg->angular_velocity.z};
        if((imu_time.seconds() - first_time) < 10) {
            imu_processor_->Imu_init(imu_data_ptr);
        }
        else
        {
            if (first_gps)
            {
                // GPS 还未到，只存 IMU 数据不融合
                last_imu_data_ptr = imu_data_ptr;
                return;
            }

            if (first_imu)
            {
                // 首帧 IMU，只存数据用于下一次预测
                first_imu = false;
                last_imu_data_ptr = imu_data_ptr;
                return;
            }

            // IMU 和 GPS 都已就绪，正常融合
            Eigen::Vector3d point_enu;
            imu_processor_->Predict(last_imu_data_ptr, imu_data_ptr, state_.get());
            auto cur_pos = state_->G_p_I;
            ConvertENUToLLA(first_lla, cur_pos, &point_enu);

            sensor_msgs::msg::NavSatFix lla;
            lla.latitude = point_enu.x();
            lla.longitude = point_enu.y();
            lla.altitude = point_enu.z();
            fusioned_gnss_pub_->publish(lla);

            last_imu_data_ptr = imu_data_ptr;
        }
    }
    void origin_rtk_yaw_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        ErrorStateKalmanFilter::GpsPositionDataPtr gps_data_ptr = std::make_shared<ErrorStateKalmanFilter::GpsPositionData>();
        double yaw_data = msg->data * M_PI / 180.0;
        gps_data_ptr->yaw = yaw_data;
        gps_data_ptr->yaw_cov = 3e-4;
        if (first_gps_yaw)
        {
            // Initialize yaw from first RTK heading observation
            state_->G_R_I = Eigen::AngleAxisd(yaw_data, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            // Update the yaw covariance to reflect measurement uncertainty
            state_->cov(8, 8) = gps_data_ptr->yaw_cov;
            first_gps_yaw = false;
            return;
        }
        gps_processor_->UpdateStateByGpsYaw(gps_data_ptr, state_.get());
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto fucion_node = std::make_shared<ESKFFusionNode>();
    rclcpp::spin(fucion_node);
    rclcpp::shutdown();
    return 0;
}
