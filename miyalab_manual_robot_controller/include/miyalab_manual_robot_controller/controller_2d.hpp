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
    std::mutex m_joy_state_mutex;
    sensor_msgs::msg::Joy::SharedPtr m_joy_state;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_state_subscriber;
    void setJoyState(const sensor_msgs::msg::Joy::SharedPtr joy);

    // ロボット速度関連
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_robot_vel_publisher;
    
    // Run状態
    bool m_is_active = false;
    std::mutex m_is_active_mutex;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_is_active_publisher;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_set_active_server;
    void serviceSetActive(const std::shared_ptr<rmw_request_id_t> header, 
                          const std_srvs::srv::SetBool::Request::SharedPtr request,
                          const std_srvs::srv::SetBool::Response::SharedPtr response);

    // 処理用
    int m_rate = -1;
    int m_active_button_num = -1;
    int m_inactive_button_num = -1;
    double m_linear_gain = -0.0;
    double m_angular_gain = -0.0;
    int m_linear_x_joy_num = -1;
    int m_linear_y_joy_num = -1;
    int m_angular_z_joy_num = -1;
    int m_slow_trigger_num = -1;
    int m_fast_trigger_num = -1;
    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MANUAL_ROBOT_CONTROLLER_2D_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------