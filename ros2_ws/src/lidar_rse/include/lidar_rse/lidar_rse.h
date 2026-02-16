#ifndef LIDAR_RSE_H
#define LIDAR_RSE_H

#include <eigen3/Eigen/Dense>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

#include "voxel.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/filters/passthrough.h>

class lidar_rse
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
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centroid_viz_pub_;

        Eigen::Affine3f body_2_inertial;
        Eigen::Affine3f rpyt2affine(
            const Eigen::VectorXd rpyt
        );
        Eigen::VectorXd rpyt;

        sensor_msgs::msg::PointCloud2 pcl;
        void pcl_callback(const sensor_msgs::msg::PointCloud2::ConstPtr msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr convert_2_voxel(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
            float leaf_size
        );

        void publishVoxelGrid(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &voxel_cloud,
            const std::string &frame_id,
            const rclcpp::Time &stamp,
            float leaf_size,
            int id = 0
        );

        std::vector<Eigen::Vector4f> clusterAndComputeCentroids(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
            double cluster_tolerance = 0.20,   // meters
            int min_cluster_size = 20,
            int max_cluster_size = 25000
        );
        void publishCentroidMarkers(
            const std::vector<Eigen::Vector4f> &centroids,
            const std::string &frame_id,
            const rclcpp::Time &stamp,
            float sphere_diameter = 0.25f,   // sphere size in meters
            const std::string &ns = "livox_frame",
            int id = 0
        );

    public:
        lidar_rse(std::shared_ptr<rclcpp::Node> node);
        ~lidar_rse();
};

#endif