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

#include "miyalab_msg_viewer/image.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using sensor_msgs::msg::Image;

//-----------------------------
// const value
//-----------------------------

//-----------------------------
// Methods
//-----------------------------
/**
 * @brief Project name
 * 
 */
namespace MiYALAB{
namespace ROS2{
/**
 * @brief Construct a new class object
 * 
 * @param options 
 */
ImageViewer::ImageViewer(rclcpp::NodeOptions options) : rclcpp::Node("laser_scan_viewer", options)
{
    // Using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    this->image = std::make_shared<Image>();
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    this->laser_subscriber = this->create_subscription<Image>("/camera/image", 10, std::bind(&ImageViewer::onImageSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    // RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    // RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Initialize Service-Client 
    // RCLCPP_INFO(this->get_logger(), "Initialize service-clients...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-clients were initialized.");

    // Main loop processing
    this->thread = std::make_unique<std::thread>(&ImageViewer::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
ImageViewer::~ImageViewer()
{
    this->thread.release();
}

void ImageViewer::onImageSubscribed(const Image::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "subscribed");
    this->image_mutex.lock();
    this->image = msg;
    this->image_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void ImageViewer::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        this->image_mutex.lock();
        auto image_ptr = this->image;
        this->image = nullptr;
        this->image_mutex.unlock();
        if(!image_ptr.get()) continue;
        if(image_ptr->data.empty()) continue;

        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", image_ptr->header.stamp.sec, image_ptr->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "encoding: %s", image_ptr->encoding.c_str());
        cv::imshow("frame", cv_bridge::toCvShare(image_ptr, image_ptr->encoding)->image);
        cv::waitKey(1);
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::ImageViewer)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------