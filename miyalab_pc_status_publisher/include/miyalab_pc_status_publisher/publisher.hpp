#ifndef __MIYALAB_ROS2_DRIVER_PC_STATUS_PUBLISHER_PUBLISHER_HPP__
#define __MIYALAB_ROS2_DRIVER_PC_STATUS_PUBLISHER_PUBLISHER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <miyalab_interfaces/msg/pc_status.hpp>

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
 * @brief PCStatusPublisher
 * 
 */
class PCStatusPublisher: public rclcpp::Node {
public:
    PCStatusPublisher(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~PCStatusPublisher();
private:
    // PCStatus
    rclcpp::Publisher<miyalab_interfaces::msg::PCStatus>::SharedPtr m_pc_status_publisher;

    // 処理用
    int m_rate = -1;
    std::string m_frame_id = "";

    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_PC_STATUS_PUBLISHER_PUBLISHER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------