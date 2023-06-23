#ifndef __MIYALAB_ROS2_OBSTACLE_DETECTOR_SIMPLE_DETECTOR_USING_POINT_CLOUD_HPP__
#define __MIYALAB_ROS2_OBSTACLE_DETECTOR_SIMPLE_DETECTOR_USING_POINT_CLOUD_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <miyalab_interfaces/msg/obstacle_array.hpp>

#include "ObstacleDetector/SimpleObstacleDetector.hpp"

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// Class
//-----------------------------
/**
 * @brief Project Name
 * 
 */
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Component Definition
 * 
 */
class SimpleObstacleDetectorUsingPointCloud: public rclcpp::Node {
public:
    SimpleObstacleDetectorUsingPointCloud(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~SimpleObstacleDetectorUsingPointCloud();
private:
 // point cloud
    std::mutex point_cloud_mutex;
    sensor_msgs::msg::PointCloud::SharedPtr point_cloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_subscriber;
    void onPointCloudSubscribed(const sensor_msgs::msg::PointCloud::SharedPtr points);

    // detector
    std::mutex detector_mutex;
    MiYALAB::Robotics::SimpleObstacleDetector detector;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr detector_range_subscriber;
    void onDetectorRangeSubscribed(const geometry_msgs::msg::Vector3::SharedPtr range);
    
    // obstacle info
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr obstacle_img_publisher;
    rclcpp::Publisher<miyalab_interfaces::msg::ObstacleArray>::SharedPtr obstacle_infos_publisher;
    
    // 処理用
    double unit;
    double add_range;
    cv::Size2d view_range;
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_OBSTACLE_DETECTOR_SIMPLE_DETECTOR_USING_POINT_CLOUD_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------