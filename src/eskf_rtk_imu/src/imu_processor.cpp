#include "eskf_rtk_imu/imu_processor.hpp"
#include <Eigen/Geometry>
#include "eskf_rtk_imu/utils.hpp"

namespace ErrorStateKalmanFilter
{
    void ImuProcessor::Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State *state)
    {
        const double delta_t = cur_imu->timestamp - last_imu->timestamp;
        const double delta_t2 = delta_t * delta_t;

        State last_state = *state;

        // mid integrate preprocess acc and gyro.
        const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + cur_imu->acc) - state->acc_bias;
        const Eigen::Vector3d gyro_unbias = 0.5 * (last_imu->gyro + cur_imu->gyro) - state->gyro_bias;

        state->G_p_I = last_state.G_p_I + last_state.G_v_I * delta_t + 0.5 * last_state.G_R_I * acc_unbias * delta_t2 + 0.5 * gravity_ * delta_t2;
        state->G_v_I = last_state.G_v_I + last_state.G_R_I * acc_unbias * delta_t + gravity_ * delta_t;

        const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;
        if (delta_angle_axis.norm() > 1e-12)
        {
            state->G_R_I = last_state.G_R_I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
        }
        else
        {
            state->G_R_I = last_state.G_R_I;
        }
        // conpute Q
        Eigen::Matrix<double, 18, 18> Qi = Eigen::Matrix<double, 18, 18>::Zero();
        Qi.block<3, 3>(3, 3) = delta_t2 * acc_noise_.asDiagonal();
        Qi.block<3, 3>(6, 6) = delta_t2 * gyro_noise_.asDiagonal();
        Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_.asDiagonal();
        Qi.block<3, 3>(12, 12) = delta_t * acc_bias_noise_.asDiagonal();

        // conpute F
        Eigen::Matrix<double, 18, 18> F_x = Eigen::Matrix<double, 18, 18>::Identity();
        F_x.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * delta_t;

        F_x.block<3, 3>(3, 6) = -state->G_R_I * GetSkewMatrix(acc_unbias) * delta_t;
        F_x.block<3, 3>(3, 12) = -state->G_R_I * delta_t;
        F_x.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * delta_t;

        if (delta_angle_axis.norm() > 1e-12)
        {
            F_x.block<3, 3>(6, 6) = Eigen::AngleAxis(-delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
        }
        F_x.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * delta_t;

        state->cov = F_x * last_state.cov * F_x.transpose() + Qi;

        state->timestamp = cur_imu->timestamp;
        state->imu_data_ptr = cur_imu;
    }
}