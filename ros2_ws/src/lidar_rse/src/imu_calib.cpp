#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <signal.h>
#include <vector>
#include <cmath>
#include <numeric>

class IMUCalibNode : public rclcpp::Node
{
public:
    IMUCalibNode() : Node("imu_calib_node")
    {
        // Subscribe to /livox/imu topic
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10,
            std::bind(&IMUCalibNode::imuCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "IMU Calibration Node started. Subscribing to /livox/imu");
        RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to calculate and print average roll/pitch");
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract linear acceleration (gravity vector when static)
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;
        
        // Calculate roll and pitch from acceleration
        double roll, pitch;
        accelerationToRollPitch(ax, ay, az, roll, pitch);
        
        // Store in buffer
        roll_buffer_.push_back(roll);
        pitch_buffer_.push_back(pitch);
        
        if (roll_buffer_.size() % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Buffered %zu samples", roll_buffer_.size());
        }
    }

    void printCalibration()
    {
        if (roll_buffer_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No IMU data received!");
            return;
        }

        // Calculate average roll and pitch
        double avg_roll = std::accumulate(roll_buffer_.begin(), roll_buffer_.end(), 0.0) / roll_buffer_.size();
        double avg_pitch = std::accumulate(pitch_buffer_.begin(), pitch_buffer_.end(), 0.0) / pitch_buffer_.size();
        
        // Calculate standard deviation
        double roll_variance = 0.0, pitch_variance = 0.0;
        for (size_t i = 0; i < roll_buffer_.size(); ++i) {
            roll_variance += std::pow(roll_buffer_[i] - avg_roll, 2);
            pitch_variance += std::pow(pitch_buffer_[i] - avg_pitch, 2);
        }
        double roll_std = std::sqrt(roll_variance / roll_buffer_.size());
        double pitch_std = std::sqrt(pitch_variance / pitch_buffer_.size());

        RCLCPP_INFO(this->get_logger(), "\n========================================");
        RCLCPP_INFO(this->get_logger(), "IMU Calibration Results");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Number of samples: %zu", roll_buffer_.size());
        RCLCPP_INFO(this->get_logger(), "Average Roll:  %.6f rad (%.3f deg) ± %.6f", 
                    avg_roll, avg_roll * 180.0 / M_PI, roll_std);
        RCLCPP_INFO(this->get_logger(), "Average Pitch: %.6f rad (%.3f deg) ± %.6f", 
                    avg_pitch, avg_pitch * 180.0 / M_PI, pitch_std);
        RCLCPP_INFO(this->get_logger(), "========================================\n");
    }

private:
    void accelerationToRollPitch(double ax, double ay, double az, 
                                 double& roll, double& pitch)
    {
        // Calculate roll and pitch from gravity vector (linear acceleration when static)
        // When the IMU is static, linear acceleration measures the gravity vector
        
        // Roll (rotation around x-axis): angle between gravity projection on yz-plane and z-axis
        roll = std::atan2(ay, az);
        
        // Pitch (rotation around y-axis): angle between gravity and yz-plane
        pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::vector<double> roll_buffer_;
    std::vector<double> pitch_buffer_;
};

// Global pointer for signal handler
std::shared_ptr<IMUCalibNode> g_node = nullptr;

void signalHandler(int signum)
{
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "\nCtrl+C detected. Calculating calibration...");
        g_node->printCalibration();
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Create node
    g_node = std::make_shared<IMUCalibNode>();
    
    // Register signal handler for Ctrl+C
    signal(SIGINT, signalHandler);
    
    // Spin
    rclcpp::spin(g_node);
    
    rclcpp::shutdown();
    return 0;
}
