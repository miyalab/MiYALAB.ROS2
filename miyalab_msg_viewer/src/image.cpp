//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

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
LaserScanViewer::LaserScanViewer(rclcpp::NodeOptions options) : rclcpp::Node("laser_scan_viewer", options)
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
    this->laser_subscriber = this->create_subscription<LaserScan>("/camera/image", 10, std::bind(&LaserScanViewer::onLaserScanSubscribed, this, _1));
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
    this->thread = std::make_unique<std::thread>(&LaserScanViewer::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
LaserScanViewer::~LaserScanViewer()
{
    this->thread.release();
}

void LaserScanViewer::onLaserScanSubscribed(const Image::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "subscribed");
    this->laser_mutex.lock();
    this->laser = msg;
    this->laser_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void LaserScanViewer::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        this->laser_mutex.lock();
        auto image_ptr = this->image;
        this->laser_mutex.unlock();

        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", image_ptr->header.stamp.sec, image_ptr->header.stamp.nanosec);

        cv::imshow("frame", frame);
        cv::waitKey(1);
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::LaserScanViewer)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------