#pragma once

#include <Eigen/Core>
#include "eskf_rtk_imu/base_type.hpp"

namespace ErrorStateKalmanFilter
{

    class GpsProcessor
    {
    public:
        GpsProcessor(const Eigen::Vector3d &I_p_Gps)
            : I_p_Gps_(I_p_Gps) {};
        bool UpdateStateByGpsPosition(const Eigen::Vector3d &init_lla,
                                      const GpsPositionDataPtr gps_data_ptr,
                                      State *state);
        bool UpdateStateByGpsYaw(const GpsPositionDataPtr gps_data_ptr,
                                 State *state);

    private:
        const Eigen::Vector3d I_p_Gps_;
        void ComputeJacobianAndResidual(const Eigen::Vector3d &init_lla,
                                        const GpsPositionDataPtr gps_data,
                                        State &state,
                                        Eigen::Matrix<double, 3, 18> *jacobian,
                                        Eigen::Vector3d *residual);
    };

}