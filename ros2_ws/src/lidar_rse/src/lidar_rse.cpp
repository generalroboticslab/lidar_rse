#include "lidar_rse/lidar_rse.h"
#include <cv_bridge/cv_bridge.h>

lidar_rse::lidar_rse(std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
    // // sensor-like QoS
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    // _pub = _node->create_publisher<sensor_msgs::msg::CompressedImage>("gimbal/image_raw/compressed", qos);

    pcl_sub = _node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar",
        10,
        std::bind(&lidar_rse::pcl_callback, this, std::placeholders::_1)
    );

    pcl_pub = _node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "rse/lidar",
        10
    );

    viz_pub = _node->create_publisher<visualization_msgs::msg::Marker>(
        "rse/grid", 
        10
    );

    centroid_viz_pub_ = _node->create_publisher<visualization_msgs::msg::Marker>("rse/cluster", 10);

    rpyt.resize(6);
}

lidar_rse::~lidar_rse()
{
    
}

void lidar_rse::pcl_callback(const sensor_msgs::msg::PointCloud2::ConstPtr msg)
{
    pcl = *msg;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud_filtered);
    rclcpp::Time cloud_stamp;
    pcl_conversions::fromPCL(cloud_filtered->header.stamp, cloud_stamp);

    rpyt << 0, 0, 0, 0, -98.0/180.0*M_PI, -180.0/180.0*M_PI;
    body_2_inertial = rpyt2affine(rpyt);
    pcl::transformPointCloud(
        *cloud_filtered,
        *cloud_filtered,
        body_2_inertial
    );

    {
        // filter out pcl too far away
        pcl::CropBox<pcl::PointXYZ> cb;

        const Eigen::Vector4f box_max(
            4.0, 
            4.0,
            2.0, 
            1
        );
        const Eigen::Vector4f box_min(
            -4.0, 
            -4.0,
            -10, 
            1
        );
        cb.setMax(box_max);
        cb.setMin(box_min);
        cb.setInputCloud(cloud_filtered);
        cb.setNegative(false);
        cb.filter(*cloud_filtered);
    }

    {
        // filter out pcl too close
        pcl::CropBox<pcl::PointXYZ> cb;

        const Eigen::Vector4f box_max(
            0.5, 
            0.5,
            5, 
            1
        );
        const Eigen::Vector4f box_min(
            -0.5, 
            -0.5,
            -5, 
            1
        );
        cb.setMax(box_max);
        cb.setMin(box_min);
        cb.setInputCloud(cloud_filtered);
        cb.setNegative(true);
        cb.filter(*cloud_filtered);
    }

    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");       // Filter along z axis
        pass.setFilterLimits(-0.8, std::numeric_limits<float>::max());
        pass.setFilterLimitsNegative(false); // Keep points within limits
        pass.filter(*cloud_filtered);
    }
    std::cout<<cloud_filtered->size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel = convert_2_voxel(cloud_filtered, 0.05);
    std::cout<<cloud_voxel->size()<<std::endl<<std::endl;;

    publishVoxelGrid(
        cloud_voxel, 
        "livox_frame", 
        cloud_stamp, 
        0.05
    );

    double cluster_tolerance = 0.25; // ~5x leaf size is fine to start
    int min_cluster_size = 2;
    int max_cluster_size = 20000;

    // 1) cluster and compute centroids
    std::vector<Eigen::Vector4f> centroids = clusterAndComputeCentroids(
        cloud_voxel, 
        cluster_tolerance, 
        min_cluster_size, 
        max_cluster_size
    );

    std::cout<<centroids.size()<<std::endl<<std::endl<<std::endl;

    // 2) publish centroids to RViz (frame and stamp from your msg)
    publishCentroidMarkers(
        centroids, 
        "livox_frame", 
        cloud_stamp, 
        0.20f
    );

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_msg);

    cloud_msg.header.frame_id = "livox_frame";       
    cloud_msg.header.stamp = cloud_stamp;  
    pcl_pub->publish(cloud_msg);
    
}


pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_rse::convert_2_voxel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    float leaf_size    // default leaf size (meters)
)
{
    auto cloud_voxel = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (!input_cloud || input_cloud->empty()) 
    {
        return cloud_voxel;
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(input_cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud_voxel);

    return cloud_voxel;
}

Eigen::Affine3f lidar_rse::rpyt2affine(const Eigen::VectorXd rpyt)
{
    Eigen::Affine3f affine_return = Eigen::Affine3f::Identity();
    if (rpyt.size()!=6)
    {
        std::cout<<"ERROR"<<std::endl;
        return affine_return;
    }

    affine_return.linear() =
        (Eigen::AngleAxisf(rpyt(3),   Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(rpyt(4), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(rpyt(5),  Eigen::Vector3f::UnitX()))
            .toRotationMatrix();

    affine_return.translation() = Eigen::Vector3f(
        rpyt(0),
        rpyt(1),
        rpyt(2)
    );

    return affine_return;
}

void lidar_rse::publishVoxelGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &voxel_cloud,
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    float leaf_size,
    int id
)
{
    if (!viz_pub || !voxel_cloud) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "livox_frame";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // scale = size of each cube (leaf size)
    marker.scale.x = leaf_size;
    marker.scale.y = leaf_size;
    marker.scale.z = leaf_size;

    // One color for all voxels (use a semi-transparent blue). You can change per-point colors by using
    // a COLOR_ARRAY marker, but CUBE_LIST uses a single color per marker unless you fill colors vector.
    std_msgs::msg::ColorRGBA color;
    color.r = 0.0f;
    color.g = 0.5f;
    color.b = 1.0f;
    color.a = 0.8f;
    marker.color = color;

    // reserve points to avoid repeated reallocations
    const size_t publish_count = std::min<size_t>(20000u, voxel_cloud->points.size());
    marker.points.reserve(publish_count);

    for (size_t i = 0; i < publish_count; ++i) {
        const auto &p = voxel_cloud->points[i];
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        marker.points.push_back(pt);
    }

    // If there were more voxels than the cap, mark the marker as partial and set lifetime short
    // if (voxel_cloud->points.size() > max_voxels_to_publish) {
    //     RCLCPP_WARN(this->get_logger(),
    //                 "publishVoxelGrid: voxel cloud size (%zu) > cap (%zu). Only publishing first %zu voxels.",
    //                 voxel_cloud->points.size(), max_voxels_to_publish, publish_count);
    //     // set a small lifetime so user sees it's partial (optional)
    //     marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    // } else {
    //     // persistent for a short time (RViz will keep if updated frequently); adjust as needed
    //     marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    // }

    viz_pub->publish(marker);
}

// Runs Euclidean clustering and returns centroids (one Eigen::Vector4f per cluster)
std::vector<Eigen::Vector4f> lidar_rse::clusterAndComputeCentroids(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double cluster_tolerance,   // meters
    int min_cluster_size,
    int max_cluster_size
)
{
    std::vector<Eigen::Vector4f> centroids;

    if (!cloud || cloud->empty()) return centroids;

    // kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    // cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // for each cluster compute centroid
    for (const auto &indices : cluster_indices) 
    {
        // make a temporary point cloud for this cluster (optional but pcl::compute3DCentroid accepts indices too)
        pcl::PointCloud<pcl::PointXYZ> cluster_points;
        cluster_points.points.reserve(indices.indices.size());
        for (int idx : indices.indices) cluster_points.points.push_back(cloud->points[idx]);
        if (cluster_points.empty()) continue;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cluster_points, centroid);
        centroids.push_back(centroid);
    }

    return centroids;
}

// publish centroids as spheres in a single Marker
void lidar_rse::publishCentroidMarkers(
    const std::vector<Eigen::Vector4f> &centroids,
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    float sphere_diameter,   // sphere size in meters
    const std::string &ns,
    int id
)
{

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // size of spheres
    marker.scale.x = sphere_diameter;
    marker.scale.y = sphere_diameter;
    marker.scale.z = sphere_diameter;

    // single color for all centroids (RGBA)
    std_msgs::msg::ColorRGBA col;
    col.r = 1.0f;   // red
    col.g = 0.0f;
    col.b = 0.0f;
    col.a = 0.9f;
    marker.color = col;

    marker.points.reserve(centroids.size());
    // If you want per-sphere colors, fill marker.colors parallel to points

    for (const auto &c : centroids)
    {
        geometry_msgs::msg::Point p;
        p.x = c[0];
        p.y = c[1];
        p.z = c[2];
        marker.points.push_back(p);
    }

    // Lifetime: if you publish every frame, small lifetime is ok. Otherwise use 0 to persist.
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    centroid_viz_pub_->publish(marker);
}