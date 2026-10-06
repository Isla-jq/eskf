#include <string.h>
#include <math.h>
#include <thread>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "wit_imu/wit_c_sdk.h"

// IMU data ready flag
static volatile char s_cDataUpdate = 0;

// Serial port fd
static int s_serial_fd = -1;

// Scale factors (from Main.cpp reference)
static constexpr float ACC_SCALE   = 16.0f / 32768.0f;   // ±16g
static constexpr float GYRO_SCALE  = 2000.0f / 32768.0f; // ±2000°/s
static constexpr float ANGLE_SCALE = 180.0f / 32768.0f;  // ±180°

// --- Linux serial port ---

static int openSerial(const std::string &port, speed_t baud)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("open serial");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);

    return fd;
}

// --- WIT SDK callbacks ---

static void serialSend(uint8_t *p_data, uint32_t uiSize)
{
    if (s_serial_fd >= 0)
        write(s_serial_fd, p_data, uiSize);
}

static void CopeSensorData(uint32_t /*uiReg*/, uint32_t /*uiRegNum*/)
{
    s_cDataUpdate = 1;
}

static void DelayMs(uint16_t ms)
{
    usleep(ms * 1000);
}

static bool autoScanSensor(const std::string &port)
{
    const int baud_rates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400};
    const speed_t baud_consts[] = {B4800, B9600, B19200, B38400, B57600, B115200, B230400};

    for (int i = 0; i < 7; i++) {
        if (s_serial_fd >= 0) {
            close(s_serial_fd);
            s_serial_fd = -1;
        }
        s_serial_fd = openSerial(port, baud_consts[i]);
        if (s_serial_fd < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("wit_imu"),
                         "Cannot open %s at %d baud", port.c_str(), baud_rates[i]);
            continue;
        }
        int retry = 2;
        do {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            usleep(100 * 1000);
            if (s_cDataUpdate != 0) {
                RCLCPP_INFO(rclcpp::get_logger("wit_imu"),
                            "Found sensor at %s, %d baud", port.c_str(), baud_rates[i]);
                return true;
            }
            retry--;
        } while (retry);
    }
    return false;
}

// --- ROS2 node ---

class WitImuNode : public rclcpp::Node
{
public:
    WitImuNode() : Node("wit_imu_node")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<std::string>("frame_id", "imu_link");

        port_ = this->get_parameter("port").as_string();
        baud_ = this->get_parameter("baud").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::SensorDataQoS());
        euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/mu/euler", rclcpp::SensorDataQoS());

        if (!setupImu()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU");
            return;
        }

        running_ = true;
        read_thread_ = std::thread(&WitImuNode::serialReadLoop, this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&WitImuNode::publishData, this));
    }

    ~WitImuNode()
    {
        running_ = false;
        if (read_thread_.joinable())
            read_thread_.join();
        if (s_serial_fd >= 0) {
            close(s_serial_fd);
            s_serial_fd = -1;
        }
    }

private:
    bool setupImu()
    {
        speed_t baud_rate = B9600;
        switch (baud_) {
            case 4800:   baud_rate = B4800;   break;
            case 9600:   baud_rate = B9600;   break;
            case 19200:  baud_rate = B19200;  break;
            case 38400:  baud_rate = B38400;  break;
            case 57600:  baud_rate = B57600;  break;
            case 115200: baud_rate = B115200; break;
            case 230400: baud_rate = B230400; break;
            case 460800: baud_rate = B460800; break;
            case 921600: baud_rate = B921600; break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unsupported baud %d, using 9600", baud_);
                baud_rate = B9600;
                break;
        }

        s_serial_fd = openSerial(port_, baud_rate);
        if (s_serial_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", port_.c_str());
            return false;
        }

        WitInit(WIT_PROTOCOL_NORMAL, 0x50);
        WitSerialWriteRegister(serialSend);
        WitRegisterCallBack(CopeSensorData);
        WitDelayMsRegister(DelayMs);

        RCLCPP_INFO(this->get_logger(), "IMU initialized");
        return true;
    }

    void serialReadLoop()
    {
        uint8_t buf[256];
        while (running_ && s_serial_fd >= 0) {
            ssize_t n = read(s_serial_fd, buf, sizeof(buf));
            if (n > 0) {
                for (ssize_t i = 0; i < n; i++)
                    WitSerialDataIn(buf[i]);
            } else if (n < 0 && errno != EAGAIN) {
                if (running_)
                    RCLCPP_ERROR(rclcpp::get_logger("wit_imu"), "Serial read error");
                break;
            } else {
                usleep(1000);
            }
        }
    }

    void publishData()
    {
        auto now = this->now();

        // --- Imu message (acceleration m/s² + angular velocity rad/s) ---
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = frame_id_;

        imu_msg.linear_acceleration.x = static_cast<float>(sReg[AX]) * ACC_SCALE * 9.7944f;
        imu_msg.linear_acceleration.y = static_cast<float>(sReg[AY]) * ACC_SCALE * 9.7944f;
        imu_msg.linear_acceleration.z = static_cast<float>(sReg[AZ]) * ACC_SCALE * 9.7944f;

        imu_msg.angular_velocity.x = static_cast<float>(sReg[GX]) * GYRO_SCALE * static_cast<float>(M_PI) / 180.0f;
        imu_msg.angular_velocity.y = static_cast<float>(sReg[GY]) * GYRO_SCALE * static_cast<float>(M_PI) / 180.0f;
        imu_msg.angular_velocity.z = static_cast<float>(sReg[GZ]) * GYRO_SCALE * static_cast<float>(M_PI) / 180.0f;

        imu_msg.orientation_covariance[0] = -1;
        imu_msg.orientation.w = 1.0;

        imu_pub_->publish(imu_msg);

        // --- Magnetic field ---
        auto mag_msg = sensor_msgs::msg::MagneticField();
        mag_msg.header.stamp = now;
        mag_msg.header.frame_id = frame_id_;
        mag_msg.magnetic_field.x = static_cast<float>(sReg[HX]);
        mag_msg.magnetic_field.y = static_cast<float>(sReg[HY]);
        mag_msg.magnetic_field.z = static_cast<float>(sReg[HZ]);
        mag_pub_->publish(mag_msg);

        // --- Euler angles (Roll, Pitch, Yaw in degrees) ---
        auto euler_msg = geometry_msgs::msg::Vector3Stamped();
        euler_msg.header.stamp = now;
        euler_msg.header.frame_id = frame_id_;
        euler_msg.vector.x = static_cast<float>(sReg[Roll]) * ANGLE_SCALE;
        euler_msg.vector.y = static_cast<float>(sReg[Pitch]) * ANGLE_SCALE;
        euler_msg.vector.z = static_cast<float>(sReg[Yaw]) * ANGLE_SCALE;
        euler_pub_->publish(euler_msg);
    }

    std::string port_;
    int baud_;
    bool autoscan_;
    std::string frame_id_;

    std::thread read_thread_;
    std::atomic<bool> running_{false};
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WitImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}