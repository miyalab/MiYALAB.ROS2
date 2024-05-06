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
#include <miyalab_interfaces/msg/computer_status.hpp>
#include <miyalab_interfaces/srv/get_computer_info.hpp>

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
 * @brief ComputerStatusPublisher
 * 
 */
class ComputerStatusPublisher: public rclcpp::Node {
public:
    ComputerStatusPublisher(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~ComputerStatusPublisher();
private:
    // PCStatus
    rclcpp::Publisher<miyalab_interfaces::msg::ComputerStatus>::SharedPtr m_computer_status_publisher;

    // PCInfoServer
    std::mutex m_computer_info_mutex;
    miyalab_interfaces::srv::GetComputerInfo::Response::SharedPtr m_computer_info_response;
    rclcpp::Service<miyalab_interfaces::srv::GetComputerInfo>::SharedPtr m_computer_info_server;
    void serviceGetComputerInfo(const std::shared_ptr<rmw_request_id_t> header, 
                            const miyalab_interfaces::srv::GetComputerInfo::Request::SharedPtr request,
                            const miyalab_interfaces::srv::GetComputerInfo::Response::SharedPtr response);

    // 処理用
    int m_rate = -1;
    std::string m_frame_id = "";
    bool m_class_cpu_file_exists;
    std::unique_ptr<std::thread> m_thread;
    void run();
    void readComputerInfo();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_PC_STATUS_PUBLISHER_PUBLISHER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------