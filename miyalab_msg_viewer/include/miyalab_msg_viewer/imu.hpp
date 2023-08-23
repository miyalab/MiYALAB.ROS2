#ifndef __MIYALAB_ROS2_DRIVER_MSG_VIEWER_IMU_VIEWER_HPP__
#define __MIYALAB_ROS2_DRIVER_MSG_VIEWER_IMU_VIEWER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

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
class ImuViewer: public rclcpp::Node {
public:
    ImuViewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~ImuViewer();
private:
    std::mutex m_mutex;
    sensor_msgs::msg::Imu::SharedPtr m_msg_ptr;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subscriber;
    void onMsgSubscribed(const sensor_msgs::msg::Imu::SharedPtr msg);

    static geometry_msgs::msg::Vector3 toRPY(const geometry_msgs::msg::Quaternion &quaternion);
    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_IMU_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------