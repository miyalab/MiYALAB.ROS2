//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

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
    m_param.device       = this->declare_parameter("camera.device", "/dev/video0");
    m_param.format       = this->declare_parameter("camera.format", "MPEG");
    m_param.frame_width  = this->declare_parameter("camera.frame.width", 1280);
    m_param.frame_height = this->declare_parameter("camera.frame.height", 720);
    m_param.fps          = this->declare_parameter("camera.fps", 30);
    m_param.rotate_flag  = this->declare_parameter("camera.rotate.right", false) 
                         - this->declare_parameter("camera.rotate.left", false);
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

    cv::VideoCapture capture(m_param.device);
    if(capture.isOpened() == false){
        RCLCPP_ERROR(this->get_logger(), "%s device not open", m_param.device.c_str());
        return;
    }
    else RCLCPP_INFO(this->get_logger(), "%s device open", m_param.device.c_str());
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, m_param.frame_height);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, m_param.frame_width);
    capture.set(cv::CAP_PROP_FPS, m_param.fps);
    if(m_param.format.size() > 4){
        capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(m_param.format[0], m_param.format[1], m_param.format[2], m_param.format[3]));
    }
    
    // Main loop
    for(rclcpp::WallRate loop(m_param.fps); rclcpp::ok(); loop.sleep()){
        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
        cv_bridge::CvImage cv_img;

        capture >> cv_img.image;
        cv_img.header.frame_id = m_param.device;
        cv_img.header.stamp = this->now();
        cv_img.encoding = "bgr8";

        if(m_param.rotate_flag != 0){
            // cv::Mat r_mat = cv::getRotationMatrix2D(cv::Point2f(cv_img.image.cols/2, cv_img.image.rows/2), 90, 1);
            // cv::warpAffine(cv_img.image, cv_img.image, r_mat, cv::Size(cv_img.image.rows, cv_img.image.cols));
            cv::rotate(cv_img.image, cv_img.image, cv::ROTATE_90_COUNTERCLOCKWISE);
            if(m_param.rotate_flag > 0) cv::flip(cv_img.image, cv_img.image, -1);
        }

        cv_img.toImageMsg(*img_msg);
        m_image_publisher->publish(std::move(img_msg));
    }

    // カメラ開放
    capture.release();
    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::Camera)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------