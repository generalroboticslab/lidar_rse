#ifndef VOXEL_H
#define VOXEL_H

#include <eigen3/Eigen/Dense>
#include <memory>
#include <pcl/filters/voxel_grid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/memory.h>   
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#define IDLE 0
#define ROTATE 0

class voxel
{
    private:
        // ROS
        std::shared_ptr<rclcpp::Node> _node;
        rclcpp::QoS qos_live = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();

        rclcpp::TimerBase::SharedPtr cam_timer;  

        double range_effiective = 5.0;
        double height_min = 0.0;


        // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_atti_pub;
        // rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gimbal_state_sub;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;

        Eigen::Affine3f body_2_inertial;
        Eigen::Affine3f rpyt2affine(
            const Eigen::VectorXd rpyt
        );
        Eigen::VectorXd rpyt;

        sensor_msgs::msg::PointCloud2 pcl;
        void pcl_callback(const sensor_msgs::msg::PointCloud2::ConstPtr msg);

    public:
        voxel(std::shared_ptr<rclcpp::Node> node);
        ~voxel();
};

#endif