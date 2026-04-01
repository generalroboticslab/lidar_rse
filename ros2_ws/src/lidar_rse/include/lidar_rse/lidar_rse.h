#ifndef LIDAR_RSE_H
#define LIDAR_RSE_H

#include <eigen3/Eigen/Dense>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include "kf.hpp"


class lidar_rse
{
    private:
        // ROS
        std::shared_ptr<rclcpp::Node> _node;
        rclcpp::QoS qos_live = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();

        double range_effiective = 5.0;
        double height_min = 0.0;
        int uav_id;


        // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_atti_pub;
        // rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gimbal_state_sub;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_raw_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centroid_viz_pub;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub;
        geometry_msgs::msg::PoseStamped ego_pose_feedback;
        bool got_ego_pose = false;
        void ego_pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
            if(!got_ego_pose)
                got_ego_pose = true;
            ego_pose_feedback = *msg;    
            ego_pose_feedback.header.frame_id="livox_frame";        
            p1_pose_pub->publish(ego_pose_feedback);
        };
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr p1_pose_pub;


        // think about how to generalize later
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr p0_pose_sub;
        geometry_msgs::msg::PoseStamped p0_pose_feedback;
        void p0_pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
            p0_pose_feedback = *msg;          
            p0_pose_feedback.header.frame_id="livox_frame";

            p0_pose_pub->publish(p0_pose_feedback);  
        };
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr p0_pose_pub;


        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr p2_pose_sub;
        geometry_msgs::msg::PoseStamped p2_pose_feedback;
        void p2_pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
            p2_pose_feedback = *msg;            
            p2_pose_feedback.header.frame_id="livox_frame";
            p2_pose_feedback.header.stamp = p0_pose_feedback.header.stamp;
            p2_pose_pub->publish(p2_pose_feedback);
        };
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr p2_pose_pub;
        

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

        void publishVoxelGrid_w_vmap(
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

        void update_voxel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

        voxel_map vmap;

        bool track_start = false;

        double time_passed = 0;

        int cluster_no;
        std::vector<Eigen::Vector4f> centroids;
        
        // helper functions and objects
        Eigen::Affine3f lidar_2_body;
        Eigen::Affine3f body_2_inertial;
        Eigen::Affine3f lidar_2_inertial;

        Eigen::Affine3f rpyt2affine(
            const Eigen::VectorXd rpyt
        );
        Eigen::VectorXd rpyt;

        Kalman3 kf;

        Eigen::Affine3f poseMsg2Affine(const geometry_msgs::msg::PoseStamped& msg)
        {
            Eigen::Affine3f T = Eigen::Affine3f::Identity();

            const auto& p = msg.pose.position;
            const auto& q = msg.pose.orientation;

            Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
            quat.normalize();

            T.linear() = quat.toRotationMatrix();
            T.translation() = Eigen::Vector3f(p.x, p.y, p.z);

            return T;
        }
        


    public:
        lidar_rse(std::shared_ptr<rclcpp::Node> node);
        ~lidar_rse();
};

#endif