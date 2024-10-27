#ifndef __MIYALAB_ROS2_DRIVER_MSG_VIEWER_VEHICLE_STATE_VIEWER_HPP__
#define __MIYALAB_ROS2_DRIVER_MSG_VIEWER_VEHICLE_STATE_VIEWER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <miyalab_interfaces/msg/vehicle_state.hpp>

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
 * @brief Odometry viewer
 * 
 */
class VehicleStateViewer: public rclcpp::Node {
public:
    VehicleStateViewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~VehicleStateViewer();
private:
    std::mutex m_mutex;
    miyalab_interfaces::msg::VehicleState::SharedPtr m_msg_ptr;
    rclcpp::Subscription<miyalab_interfaces::msg::VehicleState>::SharedPtr m_subscriber;
    void onMsgSubscribed(const miyalab_interfaces::msg::VehicleState::SharedPtr msg);
    
    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_VEHICLE_STATE_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------