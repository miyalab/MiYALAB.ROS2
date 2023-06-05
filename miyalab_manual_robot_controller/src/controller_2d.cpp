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
 * @brief Project name
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
    this->forceSet(&this->LOOP_RATE          , this->declare_parameter("controller_2d.rate", 30));
    this->forceSet(&this->ACTIVE_BUTTON_NUM  , this->declare_parameter("controller_2d.trigger.active_num", 9));
    this->forceSet(&this->INACTIVE_BUTTON_NUM, this->declare_parameter("controller_2d.trigger.inactive_mum", 10));
    this->forceSet(&this->SLOW_TRIGGER_NUM   , this->declare_parameter("controller_2d.trigger.slow_num", 5));
    this->forceSet(&this->FAST_TRIGGER_NUM   , this->declare_parameter("controller_2d.trigger.fast_num", 2));
    this->forceSet(&this->LINEAR_GAIN        , this->declare_parameter("controller_2d.linear.gain", 1.0));
    this->forceSet(&this->LINEAR_X_JOY_NUM   , this->declare_parameter("controller_2d.linear.x_num", 1));
    this->forceSet(&this->LINEAR_Y_JOY_NUM   , this->declare_parameter("controller_2d.linear.y_num", 0));
    this->forceSet(&this->ANGULAR_GAIN       , this->declare_parameter("controller_2d.angular.gain", 1.0));
    this->forceSet(&this->ANGULAR_Z_JOY_NUM  , this->declare_parameter("controller_2d.angular.z_num", 3));
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    this->joy_state_subscriber = this->create_subscription<Joy>("/joy/state", 10, std::bind(&ManualRobotController2D::setJoyState, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    this->robot_vel_publisher = this->create_publisher<Twist>("~/cmd_vel", 10);
    this->is_active_publisher = this->create_publisher<Bool>("~/is_active", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    this->set_active_server = this->create_service<SetBool>("~/set_active", std::bind(&ManualRobotController2D::serviceSetActive, this, _1, _2, _3));
    RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Initialize Service-Client 
    // RCLCPP_INFO(this->get_logger(), "Initialize service-clients...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-clients were initialized.");

    // Main loop processing
    this->thread = std::make_unique<std::thread>(&ManualRobotController2D::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the ManualRobotController2D::ManualRobotController2D object
 * 
 */
ManualRobotController2D::~ManualRobotController2D()
{
    this->thread.release();
}

void ManualRobotController2D::setJoyState(const Joy::SharedPtr joy)
{
    this->joy_state_mutex.lock();
    this->joy_state = joy;
    this->joy_state_mutex.unlock();
}

void ManualRobotController2D::serviceSetActive(const std::shared_ptr<rmw_request_id_t> header, 
                            const std_srvs::srv::SetBool::Request::SharedPtr request,
                            const std_srvs::srv::SetBool::Response::SharedPtr response)
{
    this->is_active_mutex.lock();
    this->is_active = request->data;
    response->success = this->is_active;
    this->is_active_mutex.unlock();
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
    for(rclcpp::WallRate loop(this->LOOP_RATE); rclcpp::ok(); loop.sleep()){
        // joystick入力状態 ラッチ
        this->joy_state_mutex.lock();
        const auto joy = this->joy_state;
        this->joy_state = nullptr;
        this->joy_state_mutex.unlock();
        if(!joy.get()) continue;
        if(joy->buttons.empty()) continue;

        // active状態
        this->is_active_mutex.lock();
        is_active_msg.data = this->is_active;

        // キー入力によるアクティブ状態変更
        is_active_msg.data = is_active_msg.data ||  joy->buttons[this->ACTIVE_BUTTON_NUM];
        is_active_msg.data = is_active_msg.data && !joy->buttons[this->INACTIVE_BUTTON_NUM];
        is_active_publisher->publish(is_active_msg);
        this->is_active = is_active_msg.data;
        this->is_active_mutex.unlock();
        
        // active時のみ処理
        if(is_active_msg.data){
            // 直進方向速度ゲイン
            double linear_gain = this->LINEAR_GAIN;
            double angular_gain = this->ANGULAR_GAIN;
            if(joy->axes[this->SLOW_TRIGGER_NUM] > 0.0){
                linear_gain /= 2;
                angular_gain /= 2;
            }
            if(joy->axes[this->FAST_TRIGGER_NUM] > 0.0){
                linear_gain *= 2;
                angular_gain *= 2;
            }
            Twist robot_vel_msg;
            robot_vel_msg.linear.x  = linear_gain * joy->axes[this->LINEAR_X_JOY_NUM];
            robot_vel_msg.linear.y  = linear_gain * joy->axes[this->LINEAR_Y_JOY_NUM];
            robot_vel_msg.angular.z = angular_gain * joy->axes[this->ANGULAR_Z_JOY_NUM];
            robot_vel_publisher->publish(robot_vel_msg);
        }
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