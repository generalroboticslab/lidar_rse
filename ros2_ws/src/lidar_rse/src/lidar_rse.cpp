#include "lidar_rse/lidar_rse.h"
#include <cv_bridge/cv_bridge.h>

lidar_rse::lidar_rse(std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
    // // sensor-like QoS
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    // image_pub = _node->create_publisher<sensor_msgs::msg::CompressedImage>("gimbal/image_raw/compressed", qos);

    // cv_cap.open("rtsp://192.168.144.25:8554/main.264");
    // if (!cv_cap.isOpened()) {
    //     RCLCPP_ERROR(_node->get_logger(), "Cannot open camera stream");
    //     running = false;
    //     return;
    // }
    // cv_cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // capture_thread = std::thread(&lidar_rse::cam_capture_loop, this);
    // publish_thread = std::thread(&lidar_rse::cam_publish_loop, this);
}

lidar_rse::~lidar_rse()
{
    
}
