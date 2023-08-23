#ifndef __MIYALAB_ROS2_DRIVER_JOYSTICK_JOYSTICK_HPP__
#define __MIYALAB_ROS2_DRIVER_JOYSTICK_JOYSTICK_HPP__

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
 * @brief MiYALAB ROS2
 * 
 */
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Joystick
 * 
 */
class Joystick: public rclcpp::Node {
public:
    Joystick(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~Joystick();
private:
    // joystick関連
    int m_handler = -1;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr m_state_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_is_connected_publisher;

    // 処理用
    int m_rate = -1;
    float m_dead_zone = -1.0;
    std::string m_device_path = "";

    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_JOYSTICK_JOYSTICK_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------