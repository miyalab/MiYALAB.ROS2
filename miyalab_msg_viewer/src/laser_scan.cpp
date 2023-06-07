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

#include "miyalab_msg_viewer/laser_scan.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using sensor_msgs::msg::LaserScan;

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
    this->forceSet(&this->RESOLUTION, this->declare_parameter("laser_scan_viewer.resolution", 0.5));
    this->forceSet(&this->RANGE_X, this->declare_parameter("laser_scan_viewer.range.x", 10.0));
    this->forceSet(&this->RANGE_Y, this->declare_parameter("laser_scan_viewer.range.y", 10.0));
    this->laser = std::make_shared<LaserScan>();
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    this->laser_subscriber = this->create_subscription<LaserScan>("/laser/scan", 10, std::bind(&LaserScanViewer::onLaserScanSubscribed, this, _1));
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

void LaserScanViewer::onLaserScanSubscribed(const LaserScan::SharedPtr msg)
{
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
        auto laser_cp = this->laser;
        this->laser_mutex.unlock();

        cv::Size frame_size(2*(this->RANGE_X/this->RESOLUTION)+1, 2*(this->RANGE_Y/this->RESOLUTION)+1);
        cv::Mat frame(frame_size, CV_8UC3, cv::Scalar(0,0,0));
        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", laser_cp->header.stamp.sec, laser_cp->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "range_range: %.3f - %.3f", laser_cp->range_min, laser_cp->range_max);
        RCLCPP_INFO(this->get_logger(), "angle_range: %.3f - %.3f", laser_cp->angle_min, laser_cp->angle_max);
        RCLCPP_INFO(this->get_logger(), "ranges_size: %ld", laser_cp->ranges.size());

        for(int i=0, size=laser_cp->ranges.size(); i<size; i++){
            double x = frame.cols/2 - laser_cp->ranges[i] * std::cos(laser_cp->angle_increment * i + laser_cp->angle_min) / this->RESOLUTION;
            double y = frame.rows/2 - laser_cp->ranges[i] * std::sin(laser_cp->angle_increment * i + laser_cp->angle_min) / this->RESOLUTION;

            if(x<0 || frame.cols<x) continue;
            if(y<0 || frame.rows<y) continue;

            frame.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
        }

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