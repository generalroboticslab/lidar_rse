#include "lidar_rse/lidar_rse.h"
#include <cv_bridge/cv_bridge.h>

lidar_rse::lidar_rse(std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
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

    centroid_viz_pub = _node->create_publisher<visualization_msgs::msg::Marker>("rse/cluster", 10);

    rpyt.resize(6);

    time_passed = _node->get_clock()->now().seconds();
    centroids.clear();

    Kalman3::Mat3 Q = Kalman3::Mat3::Identity() * 0.01; // process noise
    Kalman3::Mat3 R = Kalman3::Mat3::Identity() * 0.05; // measurement noise
    kf.setProcessNoise(Q);
    kf.setMeasurementNoise(R);
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

    // {
    //     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //     sor.setInputCloud(cloud_filtered);
    //     sor.setMeanK(16);                // neighborhood size (tune)
    //     sor.setStddevMulThresh(1.0);     // threshold (tune)
    //     sor.filter(*cloud_filtered);

    // };

    // heuristic filtering
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
    update_voxel(cloud_filtered);
    // vmap.remove_isolated_voxels(2);
    std::cout<<cloud_voxel->size()<<std::endl<<std::endl;;

    double cluster_tolerance = 0.4; // ~5x leaf size is fine to start
    int min_cluster_size = 3;
    int max_cluster_size = 20000;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_dyn = vmap.extract_dynamic_pcl(2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_dyn = vmap.extract_dynamic_pcl_w_intersection(10, cloud_voxel);
    publishVoxelGrid_w_vmap(
        pcl_dyn, 
        "livox_frame", 
        cloud_stamp, 
        0.05
    );
    
    std::cout<<"pcl_dyn size: "<<pcl_dyn->size()<<std::endl<<std::endl;;
    // 1) cluster and compute centroids
    centroids = clusterAndComputeCentroids(
        pcl_dyn, 
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
        0.10f
    );

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_msg);

    cloud_msg.header.frame_id = "livox_frame";       
    cloud_msg.header.stamp = cloud_stamp;  
    pcl_pub->publish(cloud_msg);
    
}


pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_rse::convert_2_voxel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    float leaf_size    
        // default leaf size (meters)
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
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
        (
            Eigen::AngleAxisf(rpyt(3),   Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(rpyt(4), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(rpyt(5),  Eigen::Vector3f::UnitX())
        ).toRotationMatrix();

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
    if (!viz_pub || !voxel_cloud) 
        return;

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
        if (c[1] < 0)
            continue;        

        if (!kf.kf_start)
        {
            Eigen::Vector3d first_pos(c[0], c[1], c[2]);
            kf.init(first_pos);
        }
        else
        {
            Eigen::Vector3d meas(c[0], c[1], c[2]);
            kf.step(meas);
        }

        geometry_msgs::msg::Point p;
        p.x = kf.state()[0];
        p.y = kf.state()[1];
        p.z = kf.state()[2];
        marker.points.push_back(p);
    }

    // Lifetime: if you publish every frame, small lifetime is ok. Otherwise use 0 to persist.
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    centroid_viz_pub->publish(marker);
}

void lidar_rse::update_voxel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
)
{
    if (!cloud || cloud->empty())
        return;

    const float hit_log_odds = 1.0f;   // tune this value

    for (const auto& pt : cloud->points)
    {
        if (!pcl::isFinite(pt))
            continue;

        // Convert point to voxel key
        VoxelKey key = vmap.key_from_point(pt.x, pt.y, pt.z);

        // Update occupancy
        
        Eigen::Vector3d pt_local(pt.x, pt.y, pt.z);

        // for 
        bool add_or_not = true;

        if (!kf.kf_start)
        {
            vmap.add_log_odds(key, hit_log_odds);
            continue;
        }

        if ((kf.state() - pt_local).norm() > 0.5)
            vmap.add_log_odds(key, 1);
        else
        {
            vmap.add_log_odds(key, -1);

        }


        // for (auto centroid : centroids)
        // {
        //     Eigen::Vector3d pt_centroid(
        //         centroid(0),
        //         centroid(1),
        //         centroid(2)
        //     );
            
        // }
        // if (add_or_not)
            


        // vmap.decay(0.8, 0.05);
        
    }

    std::cout<<"gan"<<std::endl;
    std::cout<<vmap.map.size()<<std::endl;
}

void lidar_rse::publishVoxelGrid_w_vmap(
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

    marker.scale.x = leaf_size;
    marker.scale.y = leaf_size;
    marker.scale.z = leaf_size;

    const size_t publish_count = std::min<size_t>(20000u, voxel_cloud->points.size());
    marker.points.reserve(publish_count);
    marker.colors.reserve(publish_count);   // IMPORTANT

    for (size_t i = 0; i < publish_count; ++i)
    {
        const auto &p = voxel_cloud->points[i];

        // Add voxel position
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        marker.points.push_back(pt);

        // Get log-odds from map
        VoxelKey key = vmap.key_from_point(p.x, p.y, p.z);

        float log_val = 0.0f;
        vmap.get_log_odds(key, log_val);   // if not found -> stays 0

        // Normalize [-100,100] → [0,1]
        float normalized = (log_val + 100.0f) / 200.0f;

        // normalized = (log_val) / 100.0f;
        // if (i == 0)
        //     std::cout<<normalized<<std::endl;
        normalized = std::clamp(normalized, 0.0f, 1.0f);

        std_msgs::msg::ColorRGBA color;
        color.a = 0.9f;

        // Red → Yellow → Blue gradient
        if (normalized < 0.5f)
        {
            // Red → Yellow
            float t = normalized / 0.5f;
            color.r = 1.0f;
            color.g = t;
            color.b = 0.0f;
        }
        else
        {
            // Yellow → Blue
            float t = (normalized - 0.5f) / 0.5f;
            color.r = 1.0f - t;
            color.g = 1.0f - t;
            color.b = t;
        }

        marker.colors.push_back(color);
    }

    viz_pub->publish(marker);
}