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

        void grid_establish_filter(pcl::PointCloud<pcl::PointXYZ>);
        void viz();

    public:
        voxel(std::shared_ptr<rclcpp::Node> node);
        ~voxel();
};

inline void voxel::grid_establish_filter(pcl::PointCloud<pcl::PointXYZ>)
{

}

#endif