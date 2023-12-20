//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "miyalab_msg_viewer/odometry.hpp"
#include "miyalab_ros_library/type_converter/geometry_msgs/to_rpy.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Vector3;
using geometry_msgs::msg::Quaternion;

//-----------------------------
// const value
//-----------------------------

//-----------------------------
// Methods
//-----------------------------
/**
 * @brief MiYALAB ROS2
 * 
 */
namespace MiYALAB{
namespace ROS2{
/**
 * @brief Construct a new class object
 * 
 * @param options 
 */
OdometryViewer::OdometryViewer(rclcpp::NodeOptions options) : rclcpp::Node("odom_viewer", options)
{
    // Using placeholders
    using std::placeholders::_1;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_msg_ptr = std::make_shared<Odometry>();
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_subscriber = this->create_subscription<Odometry>("/odom", 10, std::bind(&OdometryViewer::onMsgSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&OdometryViewer::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
OdometryViewer::~OdometryViewer()
{
    m_thread.release();
}

void OdometryViewer::onMsgSubscribed(const Odometry::SharedPtr msg)
{
    m_mutex.lock();
    m_msg_ptr = msg;
    m_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void OdometryViewer::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        m_mutex.lock();
        auto odom_ptr = m_msg_ptr;
        m_msg_ptr = nullptr;
        m_mutex.unlock();
        if(!odom_ptr.get()) continue;

        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", odom_ptr->header.stamp.sec, odom_ptr->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", odom_ptr->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "child_id: %s", odom_ptr->child_frame_id.c_str());

        RCLCPP_INFO(this->get_logger(), "Position     : x=%.3f, y=%.3f, z=%.3f",
            odom_ptr->pose.pose.position.x,
            odom_ptr->pose.pose.position.y,
            odom_ptr->pose.pose.position.z
        );
        RCLCPP_INFO(this->get_logger(), "Orientation  : w=%.3f, x=%.3f, y=%.3f, z=%.3f",
            odom_ptr->pose.pose.orientation.w,
            odom_ptr->pose.pose.orientation.x,
            odom_ptr->pose.pose.orientation.y,
            odom_ptr->pose.pose.orientation.z
        );
        auto odom_ptr_pose_pose_orientation = MiYALAB::ROS2::toRPY(odom_ptr->pose.pose.orientation);
        RCLCPP_INFO(this->get_logger(), "Orient(Euler): x=%.3f, y=%.3f, z=%.3f",
            odom_ptr_pose_pose_orientation.x,
            odom_ptr_pose_pose_orientation.y,
            odom_ptr_pose_pose_orientation.z
        );
        RCLCPP_INFO(this->get_logger(), "Linear vel   : x=%.3f, y=%.3f, z=%.3f",
            odom_ptr->twist.twist.linear.x,
            odom_ptr->twist.twist.linear.y,
            odom_ptr->twist.twist.linear.z
        );
        RCLCPP_INFO(this->get_logger(), "angular vel  : x=%.3f, y=%.3f, z=%.3f",
            odom_ptr->twist.twist.angular.x,
            odom_ptr->twist.twist.angular.y,
            odom_ptr->twist.twist.angular.z
        );
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::OdometryViewer)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------