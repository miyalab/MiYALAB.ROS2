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
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>

#include "miyalab_camera/camera.hpp"

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
Camera::Camera(rclcpp::NodeOptions options) : rclcpp::Node("camera", options)
{
    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_rate        = this->declare_parameter("Camera.rate", 30);
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("~/image", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&Camera::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
Camera::~Camera()
{
    m_thread.release();
}

/**
 * @brief Execute method
 * 
 */
void Camera::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());

    cv::VideoCapture capture("/dev/video0");
    
    // Main loop
    for(rclcpp::WallRate loop(m_rate); rclcpp::ok(); loop.sleep()){
        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
        cv_bridge::CvImage cv_img;

        capture >> cv_img.image;
        cv_img.header.frame_id = "cap";
        cv_img.header.stamp = this->now();
        cv_img.encoding = "bgr8";

        cv_img.toImageMsg(*img_msg);
        // RCLCPP_INFO(this->get_logger(), "address: %0x", img_msg.get());
        m_image_publisher->publish(std::move(img_msg));
        // cv::imshow("video", cv_img.image);
        // cv::waitKey(1);
        
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::Camera)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------