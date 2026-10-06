#include "eskf_rtk_imu/gps_processor.hpp"
#include "eskf_rtk_imu/utils.hpp"
#include <Eigen/Geometry>

namespace ErrorStateKalmanFilter
{
    bool GpsProcessor::UpdateStateByGpsPosition(const Eigen::Vector3d &init_lla,
                                                const GpsPositionDataPtr gps_data_ptr,
                                                State *state)
    {
        Eigen::Matrix<double, 3, 18> H;
        Eigen::Vector3d residual;
        ComputeJacobianAndResidual(init_lla, gps_data_ptr, *state, &H, &residual);
        Eigen::Matrix3d V = gps_data_ptr->cov;

        Eigen::Matrix<double, 18, 18> &P = state->cov;
        Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();

        Eigen::MatrixXd delta_x = K * residual;
        state->cov = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * P;

        state->G_p_I += delta_x.block<3, 1>(0, 0);
        state->G_v_I += delta_x.block<3, 1>(3, 0);
        if (delta_x.block<3, 1>(6, 0).norm() > 1e-12)
        {
            state->G_R_I *=
                Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
        }

        state->gyro_bias += delta_x.block<3, 1>(9, 0);
        state->acc_bias += delta_x.block<3, 1>(12, 0);
        state->gravity += delta_x.block<3, 1>(15, 0);
        return true;
    }

    bool GpsProcessor::UpdateStateByGpsYaw(const GpsPositionDataPtr gps_data_ptr,
                                           State *state)
    {
        // Compute yaw residual (wrap to [-pi, pi))
        Eigen::Vector3d euler = state->G_R_I.eulerAngles(2, 1, 0);
        double residual = gps_data_ptr->yaw - euler[0];
        residual = atan2(sin(residual), cos(residual));

        // Measurement Jacobian: H = [0,0,0, 0,0,0, 0,0,1, 0,...,0] — observes theta_z (yaw error state)
        Eigen::Matrix<double, 1, 18> H = Eigen::Matrix<double, 1, 18>::Zero();
        H(0, 8) = 1.0;

        const Eigen::Matrix<double, 18, 18> &P = state->cov;
        const double V = static_cast<double>(gps_data_ptr->yaw_cov);

        // Kalman gain (18×1 vector)
        const double S = (H * P * H.transpose())(0, 0) + V;
        // Clamp S to prevent division by zero
        if (S < 1e-12)
        {
            return false;
        }
        Eigen::Matrix<double, 18, 1> K = P * H.transpose() / S;

        // Full state correction
        Eigen::Matrix<double, 18, 1> delta_x = K * residual;

        // Apply corrections
        // Position (indices 0-2): typically zero from yaw obs alone, but keep for completeness
        state->G_p_I += delta_x.block<3, 1>(0, 0);
        // Velocity (indices 3-5)
        state->G_v_I += delta_x.block<3, 1>(3, 0);
        // Rotation / attitude (indices 6-8) — apply rotation vector
        if (delta_x.block<3, 1>(6, 0).norm() > 1e-12)
        {
            state->G_R_I *= Eigen::AngleAxisd(
                delta_x.block<3, 1>(6, 0).norm(),
                delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
        }
        // Gyro bias (indices 9-11)
        state->gyro_bias += delta_x.block<3, 1>(9, 0);
        // Acc bias (indices 12-14)
        state->acc_bias += delta_x.block<3, 1>(12, 0);
        // Gravity (indices 15-17)
        state->gravity += delta_x.block<3, 1>(15, 0);

        // Full covariance update (Joseph form for numerical stability, but (I-KH)P is sufficient here)
        state->cov = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * P;

        return true;
    }

    void GpsProcessor::ComputeJacobianAndResidual(const Eigen::Vector3d &init_lla,
                                                  const GpsPositionDataPtr gps_data,
                                                  State &state,
                                                  Eigen::Matrix<double, 3, 18> *jacobian,
                                                  Eigen::Vector3d *residual)
    {
        const Eigen::Vector3d &G_p_I = state.G_p_I;
        const Eigen::Matrix3d &G_R_I = state.G_R_I;

        Eigen::Vector3d G_p_Gps;
        ConvertLLAToENU(init_lla, gps_data->lla, &G_p_Gps);
        // RTK观测值减去IMU推算的预测值
        *residual = G_p_Gps - (G_p_I + G_R_I * I_p_Gps_);
        *jacobian = Eigen::Matrix<double, 3, 18>::Zero();

        jacobian->block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jacobian->block<3, 3>(0, 6) = -G_R_I * GetSkewMatrix(I_p_Gps_);
    }

}
