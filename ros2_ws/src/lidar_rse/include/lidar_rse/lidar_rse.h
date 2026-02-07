#ifndef LIDAR_RSE_H
#define LIDAR_RSE_H

#include <eigen3/Eigen/Dense>
#include <memory>
#include "rclcpp/rclcpp.hpp"


#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>

#define IDLE 0
#define ROTATE 0

class lidar_rse
{
    private:
        // ROS
        std::shared_ptr<rclcpp::Node> _node;
        rclcpp::QoS qos_live = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();

        rclcpp::TimerBase::SharedPtr cam_timer;    

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_rate_pub;

        void pcl_callback(const sensor_msgs::msg::CompressedImage::ConstPtr msg);

    public:
        lidar_rse(std::shared_ptr<rclcpp::Node> node);
        ~lidar_rse();
};

#endif