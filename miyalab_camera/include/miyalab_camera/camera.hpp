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
    // joystick関連
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;

    // パラメータ
    struct Param{
        std::string device;
        std::string format;
        int frame_width;
        int frame_height;
        int fps;
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