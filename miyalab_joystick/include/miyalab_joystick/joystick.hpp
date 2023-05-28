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
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr state_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_connected_publisher;

    // 処理用
    const int RATE = 1;
    const float DEAD_ZONE = -1.0;
    const std::string DEVICE_PATH = "";
    template<typename T, typename U>void forceSet(const T *value, const U &set){*((T*)value)=set;}

    std::unique_ptr<std::thread> thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_JOYSTICK_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------