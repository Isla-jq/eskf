#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "eskf_rtk_imu/base_type.hpp"
#include "eskf_rtk_imu/gps_processor.hpp"
#include "eskf_rtk_imu/imu_processor.hpp"
#include "eskf_rtk_imu/utils.hpp"

namespace ErrorStateKalmanFilter
{
    namespace
    {
        using Eigen::Matrix3d;
        using Eigen::Vector3d;

        constexpr double kGravityZ = -9.7944;
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kStatePosCov = 0.01;
        constexpr double kGpsPosCov = 0.04;
        constexpr double kGpsYawCov = 0.03;

        ImuDataPtr MakeImu(double timestamp, const Vector3d &acc, const Vector3d &gyro)
        {
            auto imu = std::make_shared<ImuData>();
            imu->timestamp = timestamp;
            imu->acc = acc;
            imu->gyro = gyro;
            return imu;
        }

        // Yaw of a rotation matrix. The atan2 form is valid over (-pi, pi]
        // whereas Eigen's eulerAngles(2, 1, 0)[0] only covers [0, pi].
        double YawOf(const Matrix3d &R)
        {
            return std::atan2(R(1, 0), R(0, 0));
        }

        Matrix3d YawRotation(double yaw)
        {
            return Eigen::AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix();
        }

        State MakeZeroState()
        {
            State state;
            state.timestamp = 0.0;
            state.lla = Vector3d::Zero();
            state.G_p_I = Vector3d::Zero();
            state.G_v_I = Vector3d::Zero();
            state.G_R_I = Matrix3d::Identity();
            state.acc_bias = Vector3d::Zero();
            state.gyro_bias = Vector3d::Zero();
            // gravity 现在是状态量，Predict 用它做传播（不再是 ImuProcessor 的常量成员）
            state.gravity = Vector3d(0.0, 0.0, kGravityZ);
            state.cov = Eigen::Matrix<double, 18, 18>::Identity() * kStatePosCov;
            state.imu_data_ptr = nullptr;
            return state;
        }

        GpsPositionDataPtr MakeGpsPosition(const Vector3d &init_lla, const Vector3d &enu)
        {
            auto gps = std::make_shared<GpsPositionData>();
            gps->timestamp = 0.0;
            ConvertENUToLLA(init_lla, enu, &gps->lla);
            gps->cov = Matrix3d::Identity() * kGpsPosCov;
            gps->yaw = 0.0;
            gps->yaw_cov = 0.0;
            return gps;
        }

        GpsPositionDataPtr MakeGpsYaw(double yaw, double yaw_cov)
        {
            auto gps = std::make_shared<GpsPositionData>();
            gps->timestamp = 0.0;
            gps->lla = Vector3d::Zero();
            gps->cov = Matrix3d::Identity() * kGpsPosCov;
            gps->yaw = yaw;
            gps->yaw_cov = yaw_cov;
            return gps;
        }
    }  // namespace

    // ------------------------------------------------------------------
    // ImuProcessor::Predict
    // ------------------------------------------------------------------

    TEST(ImuProcessorPredict, ZeroDurationKeepsState)
    {
        ImuProcessor imu_processor(0.01, 0.01, 1e-4, 1e-4);

        State state = MakeZeroState();
        state.G_v_I = Vector3d(1.0, 2.0, 3.0);
        state.cov = Eigen::Matrix<double, 18, 18>::Identity() * 1e-3;

        const ImuDataPtr sample =
            MakeImu(10.0, Vector3d(0.1, 0.2, 0.3), Vector3d(0.4, 0.5, 0.6));
        imu_processor.Predict(sample, sample, &state);

        EXPECT_NEAR(state.G_p_I.norm(), 0.0, 1e-12);
        EXPECT_NEAR((state.G_v_I - Vector3d(1.0, 2.0, 3.0)).norm(), 0.0, 1e-12);
        EXPECT_NEAR((state.G_R_I - Matrix3d::Identity()).norm(), 0.0, 1e-12);
        EXPECT_NEAR(state.cov(0, 0), 1e-3, 1e-12);
        EXPECT_NEAR(state.cov(3, 3), 1e-3, 1e-12);
    }

    TEST(ImuProcessorPredict, ConstantVelocityPropagatesPosition)
    {
        ImuProcessor imu_processor(0.01, 0.01, 1e-4, 1e-4);

        State state = MakeZeroState();
        state.G_v_I = Vector3d(1.0, 0.0, 0.0);

        const ImuDataPtr last = MakeImu(0.0, Vector3d::Zero(), Vector3d::Zero());
        const ImuDataPtr cur = MakeImu(1.0, Vector3d::Zero(), Vector3d::Zero());
        imu_processor.Predict(last, cur, &state);

        EXPECT_NEAR(state.G_p_I(0), 1.0, 1e-12);
        EXPECT_NEAR(state.G_p_I(1), 0.0, 1e-12);
        EXPECT_NEAR(state.G_p_I(2), 0.5 * kGravityZ, 1e-12);
        EXPECT_NEAR(state.G_v_I(0), 1.0, 1e-12);
        EXPECT_NEAR(state.G_v_I(2), kGravityZ, 1e-12);
        EXPECT_NEAR(state.timestamp, 1.0, 1e-15);
    }

    TEST(ImuProcessorPredict, GyroMidpointIntegrationRotatesYaw)
    {
        ImuProcessor imu_processor(0.01, 0.01, 1e-4, 1e-4);

        State state = MakeZeroState();

        // midpoint rule: (0 + pi/2) / 2 = pi/4 rad of yaw over one second
        const ImuDataPtr last = MakeImu(0.0, Vector3d::Zero(), Vector3d::Zero());
        const ImuDataPtr cur = MakeImu(1.0, Vector3d::Zero(), Vector3d(0.0, 0.0, kPi / 2.0));
        imu_processor.Predict(last, cur, &state);

        EXPECT_NEAR(YawOf(state.G_R_I), kPi / 4.0, 1e-9);
        EXPECT_NEAR((state.G_R_I * state.G_R_I.transpose() - Matrix3d::Identity()).norm(),
                    0.0, 1e-12);
    }

    TEST(ImuProcessorPredict, SubtractsAccelBias)
    {
        ImuProcessor imu_processor(0.01, 0.01, 1e-4, 1e-4);

        State state = MakeZeroState();
        // zero out gravity so that only the accelerometer bias shows up
        state.gravity = Vector3d::Zero();
        state.acc_bias = Vector3d(1.0, 0.0, 0.0);

        const ImuDataPtr last = MakeImu(0.0, Vector3d(1.0, 0.0, 0.0), Vector3d::Zero());
        const ImuDataPtr cur = MakeImu(1.0, Vector3d(1.0, 0.0, 0.0), Vector3d::Zero());
        imu_processor.Predict(last, cur, &state);

        // the measured acceleration is pure bias -> no motion at all
        EXPECT_NEAR(state.G_p_I.norm(), 0.0, 1e-12);
        EXPECT_NEAR(state.G_v_I.norm(), 0.0, 1e-12);
    }

    TEST(ImuProcessorPredict, InjectsDiscreteProcessNoise)
    {
        const double acc_noise = 0.01;
        const double gyro_noise = 0.02;
        const double acc_bias_noise = 1e-4;
        const double gyro_bias_noise = 1e-5;
        ImuProcessor imu_processor(acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise);

        State state = MakeZeroState();
        state.cov.setZero();

        const ImuDataPtr last = MakeImu(0.0, Vector3d::Zero(), Vector3d::Zero());
        const ImuDataPtr cur = MakeImu(1.0, Vector3d::Zero(), Vector3d::Zero());
        imu_processor.Predict(last, cur, &state);

        // dt = 1 -> cov == Qi == diag(..., dt^2 * acc_noise, ..., dt * bias_noise, ...)
        EXPECT_NEAR(state.cov(0, 0), 0.0, 1e-15);
        EXPECT_NEAR(state.cov(3, 3), acc_noise, 1e-15);
        EXPECT_NEAR(state.cov(6, 6), gyro_noise, 1e-15);
        EXPECT_NEAR(state.cov(9, 9), gyro_bias_noise, 1e-15);
        EXPECT_NEAR(state.cov(12, 12), acc_bias_noise, 1e-15);
    }

    // ------------------------------------------------------------------
    // GpsProcessor::UpdateStateByGpsPosition
    // ------------------------------------------------------------------

    TEST(GpsProcessorPosition, ZeroResidualKeepsNominalState)
    {
        const Vector3d init_lla(30.0, 114.0, 10.0);
        GpsProcessor gps_processor(Vector3d::Zero());

        State state = MakeZeroState();
        state.cov = Eigen::Matrix<double, 18, 18>::Identity() * kStatePosCov;

        const GpsPositionDataPtr gps_data = MakeGpsPosition(init_lla, Vector3d::Zero());
        ASSERT_TRUE(gps_processor.UpdateStateByGpsPosition(init_lla, gps_data, &state));

        // zero residual -> the nominal state is left untouched
        EXPECT_NEAR(state.G_p_I.norm(), 0.0, 1e-6);
        EXPECT_NEAR(state.G_v_I.norm(), 0.0, 1e-15);
        EXPECT_NEAR((state.G_R_I - Matrix3d::Identity()).norm(), 0.0, 1e-15);
        EXPECT_NEAR(state.gyro_bias.norm(), 0.0, 1e-15);
        EXPECT_NEAR(state.acc_bias.norm(), 0.0, 1e-15);

        // ...but information is still gained: (I - K H) P with K = P / (P + V)
        const double expected_pos_var = kStatePosCov * kGpsPosCov / (kStatePosCov + kGpsPosCov);
        EXPECT_NEAR(expected_pos_var, 0.008, 1e-12);
        EXPECT_NEAR(state.cov(0, 0), expected_pos_var, 1e-12);
        EXPECT_NEAR(state.cov(3, 3), kStatePosCov, 1e-12);
    }

    TEST(GpsProcessorPosition, PullsPositionTowardMeasurement)
    {
        const Vector3d init_lla(30.0, 114.0, 10.0);
        GpsProcessor gps_processor(Vector3d::Zero());

        State state = MakeZeroState();

        const GpsPositionDataPtr gps_data =
            MakeGpsPosition(init_lla, Vector3d(10.0, 0.0, 0.0));
        ASSERT_TRUE(gps_processor.UpdateStateByGpsPosition(init_lla, gps_data, &state));

        // the measurement is 10 m away, K = P / (P + V) = 0.01 / 0.05 = 0.2
        EXPECT_NEAR(state.G_p_I(0), 0.2 * 10.0, 1e-6);
        EXPECT_NEAR(state.G_p_I(1), 0.0, 1e-6);
        EXPECT_NEAR(state.G_p_I(2), 0.0, 1e-6);
        EXPECT_NEAR(state.G_v_I.norm(), 0.0, 1e-12);
    }

    TEST(GpsProcessorPosition, LeverArmShiftsResidual)
    {
        const Vector3d init_lla(30.0, 114.0, 10.0);
        // antenna mounted 1 m in front of the IMU
        GpsProcessor gps_processor(Vector3d(1.0, 0.0, 0.0));

        State state = MakeZeroState();

        const GpsPositionDataPtr gps_data = MakeGpsPosition(init_lla, Vector3d::Zero());
        ASSERT_TRUE(gps_processor.UpdateStateByGpsPosition(init_lla, gps_data, &state));

        // residual = 0 - (0 + I * (1,0,0)) = (-1,0,0); K_x = 0.2
        EXPECT_NEAR(state.G_p_I(0), -0.2, 1e-6);
        EXPECT_NEAR(state.G_p_I(1), 0.0, 1e-6);
        EXPECT_NEAR(state.G_p_I(2), 0.0, 1e-6);
    }

    TEST(GpsProcessorPosition, RotatedChassisRotatesLeverArm)
    {
        const Vector3d init_lla(30.0, 114.0, 10.0);
        GpsProcessor gps_processor(Vector3d(1.0, 0.0, 0.0));

        State state = MakeZeroState();
        state.G_R_I = YawRotation(kPi / 2.0);

        const GpsPositionDataPtr gps_data = MakeGpsPosition(init_lla, Vector3d::Zero());
        ASSERT_TRUE(gps_processor.UpdateStateByGpsPosition(init_lla, gps_data, &state));

        // R * (1,0,0) = (0,1,0) -> residual = (0,-1,0); K_y = 0.2
        EXPECT_NEAR(state.G_p_I(0), 0.0, 1e-6);
        EXPECT_NEAR(state.G_p_I(1), -0.2, 1e-6);
        EXPECT_NEAR(state.G_p_I(2), 0.0, 1e-6);
    }

    // ------------------------------------------------------------------
    // GpsProcessor::UpdateStateByGpsYaw
    // ------------------------------------------------------------------

    TEST(GpsProcessorYaw, CorrectsYawTowardMeasurement)
    {
        GpsProcessor gps_processor(Vector3d::Zero());

        State state = MakeZeroState();

        const GpsPositionDataPtr gps_data = MakeGpsYaw(0.1, kGpsYawCov);
        ASSERT_TRUE(gps_processor.UpdateStateByGpsYaw(gps_data, &state));

        // K = P(8,8) / (P(8,8) + V) = 0.01 / 0.04 = 0.25
        EXPECT_NEAR(YawOf(state.G_R_I), 0.25 * 0.1, 1e-9);
        EXPECT_NEAR(state.cov(8, 8), kStatePosCov * 0.75, 1e-12);

        // a yaw-only observation leaves the other nominal states untouched
        EXPECT_NEAR(state.G_p_I.norm(), 0.0, 1e-15);
        EXPECT_NEAR(state.G_v_I.norm(), 0.0, 1e-15);
        EXPECT_NEAR(state.gyro_bias.norm(), 0.0, 1e-15);
    }

    TEST(GpsProcessorYaw, WrapsResidualAcrossPiBoundary)
    {
        GpsProcessor gps_processor(Vector3d::Zero());

        State state = MakeZeroState();
        state.G_R_I = YawRotation(3.0);

        // -3.0 rad is the same heading as 3.0 rad + (2*pi - 6.0) = 3.2832 rad
        const GpsPositionDataPtr gps_data = MakeGpsYaw(-3.0, kGpsYawCov);
        ASSERT_TRUE(gps_processor.UpdateStateByGpsYaw(gps_data, &state));

        // the wrapped residual is +0.2832 rad, not the raw -6.0 rad
        const double wrapped_residual = std::atan2(std::sin(-6.0), std::cos(-6.0));
        EXPECT_NEAR(wrapped_residual, 0.2831853, 1e-6);
        EXPECT_NEAR(YawOf(state.G_R_I), 3.0 + 0.25 * wrapped_residual, 1e-9);
        EXPECT_GT(YawOf(state.G_R_I), 3.0);
        EXPECT_LT(YawOf(state.G_R_I), kPi);
    }

    TEST(GpsProcessorYaw, RejectsZeroInnovationCovariance)
    {
        GpsProcessor gps_processor(Vector3d::Zero());

        State state = MakeZeroState();
        state.cov.setZero();

        // S = P(8,8) + V = 0 -> the update has to bail out
        const GpsPositionDataPtr gps_data = MakeGpsYaw(0.5, 0.0);
        EXPECT_FALSE(gps_processor.UpdateStateByGpsYaw(gps_data, &state));

        EXPECT_NEAR(YawOf(state.G_R_I), 0.0, 1e-15);
        EXPECT_NEAR(state.cov.norm(), 0.0, 1e-15);
    }
}  // namespace ErrorStateKalmanFilter
