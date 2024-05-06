#ifndef __MIYALAB_ROS2_DRIVER_MSG_VIEWER_COMPUTER_STATUS_VIEWER_HPP__
#define __MIYALAB_ROS2_DRIVER_MSG_VIEWER_COMPUTER_STATUS_VIEWER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <miyalab_interfaces/msg/computer_status.hpp>

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
 * @brief Computer Status viewer
 * 
 */
class ComputerStatusViewer: public rclcpp::Node {
public:
    ComputerStatusViewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~ComputerStatusViewer();
private:
    std::mutex m_mutex;
    miyalab_interfaces::msg::ComputerStatus::SharedPtr m_msg_ptr;
    rclcpp::Subscription<miyalab_interfaces::msg::ComputerStatus>::SharedPtr m_subscriber;
    void onMsgSubscribed(const miyalab_interfaces::msg::ComputerStatus::SharedPtr msg);
    
    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_COMPUTER_STATUS_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------