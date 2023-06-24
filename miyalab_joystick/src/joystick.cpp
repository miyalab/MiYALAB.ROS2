//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>
#include <filesystem>

// ROS2
#include <rclcpp/rclcpp.hpp>

// linux
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include "miyalab_joystick/joystick.hpp"

//-----------------------------
// Namespace & using
//-----------------------------

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
Joystick::Joystick(rclcpp::NodeOptions options) : rclcpp::Node("joystick", options)
{
    // Using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    this->forceSet(&this->DEVICE_PATH, this->declare_parameter("joystick.path", "/dev/input/js0"));
    this->forceSet(&this->RATE       , this->declare_parameter("joystick.rate", 20));
    this->forceSet(&this->DEAD_ZONE  , this->declare_parameter("joystick.dead_zone", 0.05));
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    this->state_publisher = this->create_publisher<sensor_msgs::msg::Joy>("~/state", 10);
    this->is_connected_publisher = this->create_publisher<std_msgs::msg::Bool>("~/is_connected", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Main loop processing
    this->thread = std::make_unique<std::thread>(&Joystick::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
Joystick::~Joystick()
{
    this->thread.release();
}

/**
 * @brief Execute method
 * 
 */
void Joystick::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    std::vector<float> axes;
    std::vector<int> buttons;
    std::string device_name;
    for(rclcpp::WallRate loop(this->RATE); rclcpp::ok(); loop.sleep()){
        std_msgs::msg::Bool is_connected_msg;
        is_connected_msg.data = std::filesystem::exists(this->DEVICE_PATH);
        
        // publishデータ
        sensor_msgs::msg::Joy::UniquePtr state_msg = std::make_unique<sensor_msgs::msg::Joy>();
        state_msg->header.frame_id = device_name;
        state_msg->header.stamp    = this->now();

        // connect
        if(is_connected_msg.data && this->handler >= 0){
            //RCLCPP_INFO(this->get_logger(), "connected");
            js_event js;
            while(read(this->handler, &js, sizeof(js_event)) > 0){
                switch(js.type & ~JS_EVENT_INIT){
                case JS_EVENT_AXIS:
                    axes[js.number] = (float)js.value / 32767;
                    if (axes[js.number] < -this->DEAD_ZONE)     axes[js.number] = (axes[js.number] + this->DEAD_ZONE) / (1.0 - this->DEAD_ZONE);
                    else if(axes[js.number] >  this->DEAD_ZONE) axes[js.number] = (axes[js.number] - this->DEAD_ZONE) / (1.0 - this->DEAD_ZONE);
                    else axes[js.number] = 0;
                    break;
                case JS_EVENT_BUTTON:
                    buttons[js.number] = js.value;
                    break;
                }
            }
            state_msg->axes = axes;
            state_msg->buttons = buttons;
        }
        // connect -> disconnect
        else if(!is_connected_msg.data && this->handler >= 0){
            //RCLCPP_INFO(this->get_logger(), "disconnected");
            close(this->handler);
            this->handler = -1;
            device_name = "";
        }
        // disconnect -> connect
        else if(is_connected_msg.data && this->handler < 0){
            //RCLCPP_INFO(this->get_logger(), "connect");
            this->handler = open(this->DEVICE_PATH.c_str(), O_RDONLY);
            if(this->handler >= 0){
                int axis_size = 0;
                int button_size = 0;
                char buf[32] = "";
                ioctl(this->handler, JSIOCGAXES,     &axis_size);
                ioctl(this->handler, JSIOCGBUTTONS,  &button_size);
                ioctl(this->handler, JSIOCGNAME(32), &buf);
                axes.resize(axis_size);
                buttons.resize(button_size);
                device_name = buf;
                fcntl(this->handler, F_SETFL, O_NONBLOCK);
            }
            else RCLCPP_ERROR(this->get_logger(), "%s connect error!", DEVICE_PATH.c_str());
        }

        // publish
        this->state_publisher->publish(std::move(state_msg));
        this->is_connected_publisher->publish(is_connected_msg);
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::Joystick)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------