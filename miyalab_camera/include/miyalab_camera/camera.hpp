#ifndef __MIYALAB_ROS2_DRIVER_CAMERA_CAMERA_HPP__
#define __MIYALAB_ROS2_DRIVER_CAMERA_CAMERA_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <miyalab_interfaces/msg/camera_parameter.hpp>
#include <miyalab_interfaces/srv/get_camera_parameter.hpp>

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
 * @brief Joystick
 * 
 */
class Camera: public rclcpp::Node {
public:
    Camera(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~Camera();
private:
    // Image関連
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;

    // カメラパラメータ関連
    rclcpp::Service<miyalab_interfaces::srv::GetCameraParameter>::SharedPtr m_get_camera_parameter_service;
    void serviceGetCameraParameter(const std::shared_ptr<rmw_request_id_t> header, 
                            const miyalab_interfaces::srv::GetCameraParameter::Request::SharedPtr request,
                            const miyalab_interfaces::srv::GetCameraParameter::Response::SharedPtr response);

    // パラメータ
    struct Param{
        miyalab_interfaces::msg::CameraParameter camera_info;
        int rotate_flag; // -1: left, 0: none, 1:right
    } m_param;

    // 処理用
    int m_rate = -1;
    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_CAMERA_CAMERA_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------