#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <signal.h>
#include <cmath>

class IMUCalibNode : public rclcpp::Node
{
public:
    IMUCalibNode() : Node("imu_calib_node")
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 100,
            std::bind(&IMUCalibNode::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU Calibration Node started");
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        ax_sum_ += msg->linear_acceleration.x;
        ay_sum_ += msg->linear_acceleration.y;
        az_sum_ += msg->linear_acceleration.z;
        sample_count_++;

        if (sample_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Buffered %zu samples", sample_count_);
        }
    }

    void printCalibration()
    {
        if (sample_count_ == 0) {
            RCLCPP_WARN(this->get_logger(), "No IMU data received!");
            return;
        }

        double ax = ax_sum_ / sample_count_;
        double ay = ay_sum_ / sample_count_;
        double az = az_sum_ / sample_count_;

        double norm = std::sqrt(ax * ax + ay * ay + az * az);
        if (norm < 1e-9) {
            RCLCPP_WARN(this->get_logger(), "Acceleration norm too small!");
            return;
        }

        ax /= norm;
        ay /= norm;
        az /= norm;

        double roll = std::atan2(ay, az);
        double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

        RCLCPP_INFO(this->get_logger(), "\n========================================");
        RCLCPP_INFO(this->get_logger(), "IMU Calibration Results");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Number of samples: %zu", sample_count_);
        RCLCPP_INFO(this->get_logger(), "Mean accel unit vector: [%.6f, %.6f, %.6f]", ax, ay, az);
        RCLCPP_INFO(this->get_logger(), "Roll:  %.6f rad (%.3f deg)", roll, roll * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Pitch: %.6f rad (%.3f deg)", pitch, pitch * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "========================================\n");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    double ax_sum_ = 0.0;
    double ay_sum_ = 0.0;
    double az_sum_ = 0.0;
    size_t sample_count_ = 0;
};

std::shared_ptr<IMUCalibNode> g_node = nullptr;

void signalHandler(int signum)
{
    if (g_node) {
        g_node->printCalibration();
    }
    rclcpp::shutdown();
    std::exit(signum);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<IMUCalibNode>();
    signal(SIGINT, signalHandler);
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    return 0;
}