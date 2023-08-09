#ifndef __MIYALAB_ROS2_DRIVER_MSG_VIEWER_ODOMETRY_VIEWER_HPP__
#define __MIYALAB_ROS2_DRIVER_MSG_VIEWER_ODOMETRY_VIEWER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

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
class OdometryViewer: public rclcpp::Node {
public:
    OdometryViewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~OdometryViewer();
private:
    std::mutex odom_mutex;
    nav_msgs::msg::Odometry::SharedPtr odom;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    void onOdometrySubscribed(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    static geometry_msgs::msg::Vector3 toRPY(const geometry_msgs::msg::Quaternion &quaternion);
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_IMAGE_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------