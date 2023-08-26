//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

#include "miyalab_msg_viewer/imu.hpp"
#include "miyalab_ros_library/type_converter/geometry_msgs/to_rpy.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using sensor_msgs::msg::Imu;
using geometry_msgs::msg::Vector3;
using geometry_msgs::msg::Quaternion;

//-----------------------------
// const value
//-----------------------------
constexpr double TO_DEG = 180 / M_PI;

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
ImuViewer::ImuViewer(rclcpp::NodeOptions options) : rclcpp::Node("imu_viewer", options)
{
    // Using placeholders
    using std::placeholders::_1;

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_subscriber = this->create_subscription<Imu>("/imu", 10, std::bind(&ImuViewer::onMsgSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&ImuViewer::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
ImuViewer::~ImuViewer()
{
    m_thread.release();
}

void ImuViewer::onMsgSubscribed(const Imu::SharedPtr msg)
{
    m_mutex.lock();
    m_msg_ptr = msg;
    m_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void ImuViewer::run()
{
RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        m_mutex.lock();
        auto imu_ptr = m_msg_ptr;
        m_msg_ptr = nullptr;
        m_mutex.unlock();
        if(!imu_ptr.get()) continue;

        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "header: id=%s [%d.%09d]",
            imu_ptr->header.frame_id.c_str(),
            imu_ptr->header.stamp.sec,
            imu_ptr->header.stamp.nanosec
        );
        RCLCPP_INFO(this->get_logger(), "acceleration : x=%.3f, y=%.3f, z=%.3f",
            imu_ptr->linear_acceleration.x,
            imu_ptr->linear_acceleration.y,
            imu_ptr->linear_acceleration.z
        );
        RCLCPP_INFO(this->get_logger(), "angular vel  : x=%.3f, y=%.3f, z=%.3f",
            imu_ptr->angular_velocity.x,
            imu_ptr->angular_velocity.y,
            imu_ptr->angular_velocity.z
        );
        RCLCPP_INFO(this->get_logger(), "Orientation  : w=%.3f, x=%.3f, y=%.3f, z=%.3f",
            imu_ptr->orientation.w,
            imu_ptr->orientation.x,
            imu_ptr->orientation.y,
            imu_ptr->orientation.z
        );
        auto imu_ptr_orientation = MiYALAB::ROS2::toRPY(imu_ptr->orientation);
        RCLCPP_INFO(this->get_logger(), "Orient(Euler): x=%.3f, y=%.3f, z=%.3f",
            imu_ptr_orientation.x,
            imu_ptr_orientation.y,
            imu_ptr_orientation.z
        );
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::ImuViewer)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------