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

    // cv_cap.open("rtsp://192.168.144.25:8554/main.264");
    // if (!cv_cap.isOpened()) {
    //     RCLCPP_ERROR(_node->get_logger(), "Cannot open camera stream");
    //     running = false;
    //     return;
    // }
    // cv_cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // capture_thread = std::thread(&lidar_rse::cam_capture_loop, this);
    // publish_thread = std::thread(&lidar_rse::cam_publish_loop, this);

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

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_msg);

    cloud_msg.header.frame_id = "livox_frame";          // or your frame
    cloud_msg.header.stamp = cloud_stamp;       // from earlier
    pcl_pub->publish(cloud_msg);
    
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

    // translation
    affine_return.translation() = Eigen::Vector3f(
        rpyt(0),
        rpyt(1),
        rpyt(2)
    );


}