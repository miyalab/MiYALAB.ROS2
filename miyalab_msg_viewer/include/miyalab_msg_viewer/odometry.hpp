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
 * @brief MiYALAB ROS2
 * 
 */
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Odometry viewer
 * 
 */
class OdometryViewer: public rclcpp::Node {
public:
    OdometryViewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~OdometryViewer();
private:
    std::mutex m_mutex;
    nav_msgs::msg::Odometry::SharedPtr m_msg_ptr;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber;
    void onMsgSubscribed(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_IMAGE_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------