#include <thread>
#include <sensor_msgs/msg/imu.hpp>

class ImuQueue
{
public:
    void push(const sensor_msgs::msg::Imu::SharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(msg);
    }

    bool pop(sensor_msgs::msg::Imu::SharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        msg = queue_.front();
        queue_.pop();
        return true;
    }

private:
    std::mutex mutex_;
    std::queue<sensor_msgs::msg::Imu::SharedPtr> queue_;
};