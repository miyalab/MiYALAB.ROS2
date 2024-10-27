//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "miyalab_msg_viewer/vehicle_state.hpp"
#include "miyalab_ros_library/type_converter/geometry_msgs/to_rpy.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using miyalab_interfaces::msg::VehicleState;

//-----------------------------
// const value
//-----------------------------

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
 * @brief Construct a new class object
 * 
 * @param options 
 */
VehicleStateViewer::VehicleStateViewer(rclcpp::NodeOptions options) : rclcpp::Node("odom_viewer", options)
{
    // Using placeholders
    using std::placeholders::_1;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_msg_ptr = std::make_shared<VehicleState>();
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_subscriber = this->create_subscription<VehicleState>("/state", 10, std::bind(&VehicleStateViewer::onMsgSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&VehicleStateViewer::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
VehicleStateViewer::~VehicleStateViewer()
{
    m_thread.release();
}

void VehicleStateViewer::onMsgSubscribed(const VehicleState::SharedPtr msg)
{
    m_mutex.lock();
    m_msg_ptr = msg;
    m_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void VehicleStateViewer::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        m_mutex.lock();
        auto msg_ptr = m_msg_ptr;
        m_msg_ptr = nullptr;
        m_mutex.unlock();
        if(!msg_ptr.get()) continue;

        std::string info = "";
        RCLCPP_INFO(this->get_logger(), "frame id: %s", msg_ptr->header.frame_id.c_str());

        info = "Encoder: ";
        for(int i=0, size=msg_ptr->encoder.size(); i<size; i++) info += std::to_string(msg_ptr->encoder[i]) + "\t";
        RCLCPP_INFO(this->get_logger(), info.c_str());

        info = "Voltage: ";
        for(int i=0, size=msg_ptr->motor_voltage.size(); i<size; i++) info += std::to_string(msg_ptr->motor_voltage[i]) + "\t";
        RCLCPP_INFO(this->get_logger(), info.c_str());

        info = "Current: ";
        for(int i=0, size=msg_ptr->motor_current.size(); i<size; i++) info += std::to_string(msg_ptr->motor_current[i]) + "\t";
        RCLCPP_INFO(this->get_logger(), info.c_str());

        info = "PwmDuty: ";
        for(int i=0, size=msg_ptr->motor_pwm.size(); i<size; i++) info += std::to_string(msg_ptr->motor_pwm[i]) + "\t";
        RCLCPP_INFO(this->get_logger(), info.c_str());

        info = "RPM    : ";
        for(int i=0, size=msg_ptr->motor_rpm.size(); i<size; i++) info += std::to_string(msg_ptr->motor_rpm[i]) + "\t";
        RCLCPP_INFO(this->get_logger(), info.c_str());

        info = "Temp   : ";
        for(int i=0, size=msg_ptr->motor_temperature.size(); i<size; i++) info += std::to_string(msg_ptr->motor_temperature[i]) + "\t";
        RCLCPP_INFO(this->get_logger(), info.c_str());

        RCLCPP_INFO(this->get_logger(), "---");

    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::VehicleStateViewer)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------