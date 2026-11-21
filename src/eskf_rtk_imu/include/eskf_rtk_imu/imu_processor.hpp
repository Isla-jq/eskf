#pragma once

#include "eskf_rtk_imu/base_type.hpp"
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

namespace ErrorStateKalmanFilter
{
    class ImuProcessor
    {
    public:
    ImuProcessor(const double acc_noise, const double gyro_noise,
                 const double acc_bias_noise, const double gyro_bias_noise)
                 :
                 acc_noise_({acc_noise, acc_noise, acc_noise}),
                 gyro_noise_({gyro_noise, gyro_noise, gyro_noise}),
                 acc_bias_noise_({acc_bias_noise, acc_bias_noise, acc_bias_noise}),
                 gyro_bias_noise_({gyro_bias_noise, gyro_bias_noise, gyro_bias_noise}),
                 N(100){};

    void Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state);

    // 静止初始化阶段累积出来的均值，供外部初始化零偏使用
    Eigen::Vector3d GetMeanAcc() const { return mean_acc_; }
    Eigen::Vector3d GetMeanGyro() const { return mean_gyr_; }

    void Imu_init(ImuDataPtr imuData) {
        if (b_first_frame_)
        {
            b_first_frame_ = false;
            mean_acc_ = imuData->acc;
            mean_gyr_ = imuData->gyro;
            return;
        }
     
        Eigen::Vector3d cur_acc = imuData->acc;
        Eigen::Vector3d cur_gyr = imuData->gyro;
        mean_acc_      += (cur_acc - mean_acc_) / N;
        mean_gyr_      += (cur_gyr - mean_gyr_) / N;
        acc_noise_ = acc_noise_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        gyro_noise_ = gyro_noise_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        const double MIN_GYRO_NOISE = 1e-4;
        gyro_noise_ = gyro_noise_.cwiseMax(Eigen::Vector3d(MIN_GYRO_NOISE, MIN_GYRO_NOISE, MIN_GYRO_NOISE));
        // RCLCPP_WARN(rclcpp::get_logger("main"),"%f %f %f\nR",acc_noise_[0],acc_noise_[1], acc_noise_[2]);
        // RCLCPP_WARN(rclcpp::get_logger("mian"),"%f %f %f\n",gyro_noise_[0]*10,gyro_noise_[1],gyro_noise_[2]);
        // RCLCPP_WARN(rclcpp::get_logger("main"),"%f %f %f\n",acc_bias_noise_[0],acc_bias_noise_[1], acc_bias_noise_[2]);
        // RCLCPP_WARN(rclcpp::get_logger("mian"),"%f %f %f\n\n",gyro_bias_noise_[0],gyro_bias_noise_[1], gyro_bias_noise_[2]);
    }

    private:

    bool   b_first_frame_ = true;
    Eigen::Vector3d mean_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d acc_noise_;
    Eigen::Vector3d gyro_noise_;
    Eigen::Vector3d acc_bias_noise_;
    Eigen::Vector3d gyro_bias_noise_;
    // const double acc_noise_;
    // const double gyro_noise_;
    // const double acc_bias_noise_;
    // const double gyro_bias_noise_;
    int N;
    };
}