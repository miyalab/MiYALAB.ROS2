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
 * @brief Project name
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

Vector3 ImuViewer::toRPY(const Quaternion &quaternion)
{
    geometry_msgs::msg::Vector3 ret;

    const double q0q0 = quaternion.w * quaternion.w;
	const double q1q1 = quaternion.x * quaternion.x;
	const double q2q2 = quaternion.y * quaternion.y;
	const double q3q3 = quaternion.z * quaternion.z;
	const double q0q1 = quaternion.w * quaternion.x;
	const double q0q2 = quaternion.w * quaternion.y;
	const double q0q3 = quaternion.w * quaternion.z;
	const double q1q2 = quaternion.x * quaternion.y;
	const double q1q3 = quaternion.x * quaternion.z;
	const double q2q3 = quaternion.y * quaternion.z;

    ret.x = std::atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    ret.y = -std::asin(2.0 * (q1q3 - q0q2));
    ret.z = std::atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);

    return ret;
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
        auto imu_ptr_orientation = this->toRPY(imu_ptr->orientation);
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