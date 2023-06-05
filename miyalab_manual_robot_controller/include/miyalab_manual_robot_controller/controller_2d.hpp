#ifndef __MIYALAB_ROS2_DRIVER_MANUAL_ROBOT_CONTROLLER_2D_HPP__
#define __MIYALAB_ROS2_DRIVER_MANUAL_ROBOT_CONTROLLER_2D_HPP__

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
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
class ManualRobotController2D: public rclcpp::Node {
public:
    ManualRobotController2D(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~ManualRobotController2D();
private:
    // joystick関連
    std::mutex joy_state_mutex;
    sensor_msgs::msg::Joy::SharedPtr joy_state;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_state_subscriber;
    void setJoyState(const sensor_msgs::msg::Joy::SharedPtr joy);

    // ロボット速度関連
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_vel_publisher;
    
    // Run状態
    bool is_active = false;
    std::mutex is_active_mutex;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_active_publisher;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_active_server;
    void serviceSetActive(const std::shared_ptr<rmw_request_id_t> header, 
                          const std_srvs::srv::SetBool::Request::SharedPtr request,
                          const std_srvs::srv::SetBool::Response::SharedPtr response);

    // 処理用
    const int LOOP_RATE = -1;
    const int ACTIVE_BUTTON_NUM = -1;
    const int INACTIVE_BUTTON_NUM = -1;
    const double LINEAR_GAIN = -0.0;
    const double ANGULAR_GAIN = -0.0;
    const int LINEAR_X_JOY_NUM = -1;
    const int LINEAR_Y_JOY_NUM = -1;
    const int ANGULAR_Z_JOY_NUM = -1;
    const int SLOW_TRIGGER_NUM = -1;
    const int FAST_TRIGGER_NUM = -1;
    template<typename T, typename U>void forceSet(const T *value, const U &set){*((T*)value)=set;}
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MANUAL_ROBOT_CONTROLLER_2D_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------