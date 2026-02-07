#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lidar_rse/lidar_rse.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // enable intra-process communication for zero-copy
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("LIDAR_RSE", options);
    lidar_rse lidar_rse_node(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
