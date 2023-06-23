#ifndef __MIYALAB_ROS2_DRIVER_MSG_VIEWER_IMAGE_VIEWER_HPP__
#define __MIYALAB_ROS2_DRIVER_MSG_VIEWER_IMAGE_VIEWER_HPP__

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

// OpenCV
#include <opencv2/opencv.hpp>

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// Class
//-----------------------------
/**
 * @brief Project Name
 * 
 */
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Component Definition
 * 
 */
class ImageViewer: public rclcpp::Node {
public:
    ImageViewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~ImageViewer();
private:
    std::mutex image_mutex;
    sensor_msgs::msg::Image::SharedPtr image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr laser_subscriber;
    void onImageSubscribed(const sensor_msgs::msg::Image::SharedPtr msg);
    
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_IMAGE_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------