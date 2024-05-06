//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "miyalab_msg_viewer/computer_status.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using miyalab_interfaces::msg::ComputerStatus;

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
ComputerStatusViewer::ComputerStatusViewer(rclcpp::NodeOptions options) : rclcpp::Node("computer_status_viewer", options)
{
    // Using placeholders
    using std::placeholders::_1;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_msg_ptr = std::make_shared<ComputerStatus>();
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_subscriber = this->create_subscription<ComputerStatus>("/computer/status", 10, std::bind(&ComputerStatusViewer::onMsgSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&ComputerStatusViewer::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
ComputerStatusViewer::~ComputerStatusViewer()
{
    m_thread.release();
}

void ComputerStatusViewer::onMsgSubscribed(const ComputerStatus::SharedPtr msg)
{
    m_mutex.lock();
    m_msg_ptr = msg;
    m_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void ComputerStatusViewer::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        m_mutex.lock();
        auto msg_ptr = m_msg_ptr;
        m_msg_ptr = nullptr;
        m_mutex.unlock();
        if(!msg_ptr.get()) continue;

        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", msg_ptr->header.stamp.sec, msg_ptr->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", msg_ptr->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "cpu usage: %lf", msg_ptr->cpu_usage);
        std::string list = "[";
        for(int i=0, size=msg_ptr->cpu_freq.size(); i<size; i++){
            char buf[32] = "";
            std::snprintf(buf, sizeof(buf), "%.0lf, ", msg_ptr->cpu_freq[i]);
            list += buf;
        }
        list.pop_back();
        list[list.size()-1] = ']';
        RCLCPP_INFO(this->get_logger(), "cpu freq : %s", list.c_str());
        RCLCPP_INFO(this->get_logger(), "mem usage: %lf", msg_ptr->memory_usage);
        RCLCPP_INFO(this->get_logger(), "Temp");
        for(const auto &temp: msg_ptr->temperatures){
            RCLCPP_INFO(this->get_logger(), " - %s: %lf", temp.header.frame_id.c_str(), temp.temperature);
        }

    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::ComputerStatusViewer)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------