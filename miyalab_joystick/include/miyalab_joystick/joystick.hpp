#ifndef __MIYALAB_ROS2_DRIVER_JOYSTICK_HPP__
#define __MIYALAB_ROS2_DRIVER_JOYSTICK_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>

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
class Joystick: public rclcpp::Node {
public:
    Joystick(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~Joystick();
private:
    // joystick関連
    int handler = -1;
    std::string device_name;
    std::string device_path;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr state_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_connected_publisher;

    // 処理用
    int rate;
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_JOYSTICK_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------