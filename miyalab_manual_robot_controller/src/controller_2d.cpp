//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "miyalab_manual_robot_controller/controller_2d.hpp"

//-----------------------------
// Namespace & using
//-----------------------------
using std_msgs::msg::Bool;
using std_srvs::srv::SetBool;
using sensor_msgs::msg::Joy;
using geometry_msgs::msg::Twist;

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
 * @brief Construct a new ManualRobotController2D::ManualRobotController2D object
 * 
 * @param options 
 */
ManualRobotController2D::ManualRobotController2D(rclcpp::NodeOptions options) : rclcpp::Node("robot_run", options)
{
    // Using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_rate               = this->declare_parameter("controller_2d.rate", 30);
    m_active_button_num  = this->declare_parameter("controller_2d.trigger.active_num", 9);
    m_inactive_button_num= this->declare_parameter("controller_2d.trigger.inactive_mum", 10);
    m_slow_trigger_num   = this->declare_parameter("controller_2d.trigger.slow_num", 5);
    m_fast_trigger_num   = this->declare_parameter("controller_2d.trigger.fast_num", 2);
    m_linear_gain        = this->declare_parameter("controller_2d.linear.gain", 1.0);
    m_linear_x_joy_num   = this->declare_parameter("controller_2d.linear.x_num", 1);
    m_linear_y_joy_num   = this->declare_parameter("controller_2d.linear.y_num", 0);
    m_angular_gain       = this->declare_parameter("controller_2d.angular.gain", 1.0);
    m_angular_z_joy_num  = this->declare_parameter("controller_2d.angular.z_num", 3);
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_joy_state_subscriber = this->create_subscription<Joy>("/joy/state", 10, std::bind(&ManualRobotController2D::setJoyState, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    m_robot_vel_publisher = this->create_publisher<Twist>("~/cmd_vel", 10);
    m_is_active_publisher = this->create_publisher<Bool>("~/is_active", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    m_set_active_server = this->create_service<SetBool>("~/set_active", std::bind(&ManualRobotController2D::serviceSetActive, this, _1, _2, _3));
    RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&ManualRobotController2D::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the ManualRobotController2D::ManualRobotController2D object
 * 
 */
ManualRobotController2D::~ManualRobotController2D()
{
    m_thread.release();
}

void ManualRobotController2D::setJoyState(const Joy::SharedPtr joy)
{
    m_joy_state_mutex.lock();
    m_joy_state = joy;
    m_joy_state_mutex.unlock();
}

void ManualRobotController2D::serviceSetActive(const std::shared_ptr<rmw_request_id_t> header, 
                            const std_srvs::srv::SetBool::Request::SharedPtr request,
                            const std_srvs::srv::SetBool::Response::SharedPtr response)
{
    m_is_active_mutex.lock();
    m_is_active = request->data;
    response->success = m_is_active;
    m_is_active_mutex.unlock();
    response->message = "ok";
}

/**
 * @brief Execute method
 * 
 */
void ManualRobotController2D::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    std_msgs::msg::Bool is_active_msg;
    
    // Main loop
    for(rclcpp::WallRate loop(m_rate); rclcpp::ok(); loop.sleep()){
        // joystick入力状態 ラッチ
        m_joy_state_mutex.lock();
        const auto joy_ptr = m_joy_state;
        m_joy_state = nullptr;
        m_joy_state_mutex.unlock();

        // active状態
        m_is_active_mutex.lock();
        is_active_msg.data = m_is_active;
        m_is_active_mutex.unlock();

        // joystickデータ受信 かつ データが存在
        if(joy_ptr.get() && !joy_ptr->buttons.empty()){
            // アクティブ状態変更
            is_active_msg.data = is_active_msg.data ||  joy_ptr->buttons[m_active_button_num];
            is_active_msg.data = is_active_msg.data && !joy_ptr->buttons[m_inactive_button_num];

            // アクティブ時のみ処理
            if(is_active_msg.data){
                // 直進方向速度ゲイン
                double linear_gain = this->m_linear_gain;
                double angular_gain = this->m_angular_gain;
                if(joy_ptr->axes[this->m_slow_trigger_num] > 0.0){
                    linear_gain /= 2;
                    angular_gain /= 2;
                }
                if(joy_ptr->axes[this->m_fast_trigger_num] > 0.0){
                    linear_gain *= 2;
                    angular_gain *= 2;
                }
                Twist robot_vel_msg;
                robot_vel_msg.linear.x  = linear_gain * joy_ptr->axes[this->m_linear_x_joy_num];
                robot_vel_msg.linear.y  = linear_gain * joy_ptr->axes[this->m_linear_y_joy_num];
                robot_vel_msg.angular.z = angular_gain * joy_ptr->axes[this->m_angular_z_joy_num];
                m_robot_vel_publisher->publish(robot_vel_msg);
            }
        }

        m_is_active_mutex.lock();
        m_is_active = is_active_msg.data;
        m_is_active_mutex.unlock();

        // アクティブ状態
        m_is_active_publisher->publish(is_active_msg);
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::ManualRobotController2D)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------