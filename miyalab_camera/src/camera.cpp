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
#include <miyalab_ros_library/type_converter/geometry_msgs/to_quaternion.hpp>

//-----------------------------
// using & namespace
//-----------------------------
using sensor_msgs::msg::Image;
using miyalab_interfaces::msg::CameraParameter;
using miyalab_interfaces::srv::GetCameraParameter;

//-----------------------------
// Const Value
//-----------------------------
constexpr double TO_RAD = M_PI / 180.0;

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
    // using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_param.camera_info.device             = this->declare_parameter("camera.device", "/dev/video0");
    m_param.camera_info.codec              = this->declare_parameter("camera.codec", "MPEG");
    m_param.camera_info.image_size.x       = this->declare_parameter("camera.frame.width", 1280);
    m_param.camera_info.image_size.y       = this->declare_parameter("camera.frame.height", 720);
    m_param.camera_info.angle_of_view.x    = this->declare_parameter("camera.angle_of_view.horizontal", 74.0) * TO_RAD;
    m_param.camera_info.angle_of_view.y    = this->declare_parameter("camera.angle_of_view.vertical", 42.0) * TO_RAD;
    m_param.camera_info.fps                = this->declare_parameter("camera.fps", 30);
    m_param.camera_info.offset.position.x  = this->declare_parameter("camera.offset.position.x", 0.0);
    m_param.camera_info.offset.position.y  = this->declare_parameter("camera.offset.position.y", 0.0);
    m_param.camera_info.offset.position.z  = this->declare_parameter("camera.offset.position.z", 0.0);
    auto pitch                             = this->declare_parameter("camera.offset.orientation.pitch", 0.0) * TO_RAD;
    auto yaw                               = this->declare_parameter("camera.offset.orientation.yaw", 0.0) * TO_RAD;
    m_param.camera_info.offset.orientation = MiYALAB::ROS2::toQuaternion(0, pitch, yaw);
    m_param.rotate_flag  = this->declare_parameter("camera.rotate.right", false) 
                         - this->declare_parameter("camera.rotate.left", false);
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    m_image_publisher = this->create_publisher<Image>("~/image", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service
    RCLCPP_INFO(this->get_logger(), "Initialize services...");
    m_get_camera_parameter_service = this->create_service<GetCameraParameter>("~/get_camera_parameter", std::bind(&Camera::serviceGetCameraParameter, this, _1, _2, _3));
    RCLCPP_INFO(this->get_logger(), "Complete! Services were initialized.");

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

void Camera::serviceGetCameraParameter(const std::shared_ptr<rmw_request_id_t> header, 
                                       const GetCameraParameter::Request::SharedPtr request,
                                       const GetCameraParameter::Response::SharedPtr response)
{
    response->parameter = m_param.camera_info;
    if(m_param.rotate_flag){
        std::swap(response->parameter.image_size.x, response->parameter.image_size.y);
        std::swap(response->parameter.angle_of_view.x, response->parameter.angle_of_view.y);
    }
    response->message = "ok";
    response->success = true;
}

/**
 * @brief Execute method
 * 
 */
void Camera::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());

    // setup capture device
    cv::VideoCapture capture(m_param.camera_info.device);
    if(capture.isOpened() == false){
        RCLCPP_ERROR(this->get_logger(), "%s device not open", m_param.camera_info.device.c_str());
        return;
    }
    else RCLCPP_INFO(this->get_logger(), "%s device open", m_param.camera_info.device.c_str());
    capture.set(cv::CAP_PROP_FRAME_WIDTH,  m_param.camera_info.image_size.x);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, m_param.camera_info.image_size.y);
    capture.set(cv::CAP_PROP_FPS, m_param.camera_info.fps);
    if(m_param.camera_info.codec.size() > 4){
        capture.set(
            cv::CAP_PROP_FOURCC, 
            cv::VideoWriter::fourcc(
                m_param.camera_info.codec[0], 
                m_param.camera_info.codec[1],
                m_param.camera_info.codec[2], 
                m_param.camera_info.codec[3]
            )
        );
    }
    
    // Main loop
    for(rclcpp::WallRate loop(m_param.camera_info.fps); rclcpp::ok(); loop.sleep()){
        // make instance
        auto img_msg = std::make_unique<Image>();
        cv_bridge::CvImage cv_img;

        // get image
        capture >> cv_img.image;
        cv_img.header.frame_id = m_param.camera_info.device;
        cv_img.header.stamp = this->now();
        cv_img.encoding = sensor_msgs::image_encodings::BGR8;

        // image rotation
        if(m_param.rotate_flag < 0)      cv::rotate(cv_img.image, cv_img.image, cv::ROTATE_90_COUNTERCLOCKWISE);
        else if(m_param.rotate_flag > 0) cv::rotate(cv_img.image, cv_img.image, cv::ROTATE_90_CLOCKWISE);

        // publish
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