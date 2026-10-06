#pragma once

#include <memory>
#include <Eigen/Core>

namespace ErrorStateKalmanFilter
{
    struct ImuData
    {
        double timestamp;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
    };

    using ImuDataPtr = std::shared_ptr<ImuData>;

    struct GpsPositionData
    {
        double timestamp;
        Eigen::Vector3d lla;
        Eigen::Matrix3d cov;
        double yaw;
        double yaw_cov;
    };
    using GpsPositionDataPtr = std::shared_ptr<GpsPositionData>;

    struct State
    {
        double timestamp;

        Eigen::Vector3d lla;
        Eigen::Vector3d G_p_I;
        Eigen::Vector3d G_v_I;
        Eigen::Matrix3d G_R_I;
        Eigen::Vector3d acc_bias;
        Eigen::Vector3d gyro_bias;
        Eigen::Vector3d gravity;

        Eigen::Matrix<double, 18, 18> cov;
        ImuDataPtr imu_data_ptr;
    };

}