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
// const value
//-----------------------------
constexpr double TO_DEG = 180 / M_PI;

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

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_resolution = this->declare_parameter("laser_scan_viewer.resolution", 0.5);
    m_range_x    = this->declare_parameter("laser_scan_viewer.range.x", 10.0);
    m_range_y    = this->declare_parameter("laser_scan_viewer.range.y", 10.0);
    m_background_color[0] = this->declare_parameter("laser_scan_viewer.background_color.b", 0);
    m_background_color[1] = this->declare_parameter("laser_scan_viewer.background_color.g", 0);
    m_background_color[2] = this->declare_parameter("laser_scan_viewer.background_color.r", 0);
    m_point_color[0] = this->declare_parameter("laser_scan_viewer.point_color.b", 255);
    m_point_color[1] = this->declare_parameter("laser_scan_viewer.point_color.g", 255);
    m_point_color[2] = this->declare_parameter("laser_scan_viewer.point_color.r", 255);
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_subscriber = this->create_subscription<LaserScan>("/laser/scan", 10, std::bind(&LaserScanViewer::onMsgSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&LaserScanViewer::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
LaserScanViewer::~LaserScanViewer()
{
    m_thread.release();
}

void LaserScanViewer::onMsgSubscribed(const LaserScan::SharedPtr msg)
{
    m_mutex.lock();
    m_msg_ptr = msg;
    m_mutex.unlock();
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
        m_mutex.lock();
        auto msg_ptr = m_msg_ptr;
        m_msg_ptr = nullptr;
        m_mutex.unlock();
        if(!msg_ptr.get()) continue;

        cv::Size frame_size(2*(m_range_x/m_resolution)+1, 2*(m_range_y/m_resolution)+1);
        cv::Mat frame(frame_size, CV_8UC3, this->m_background_color);
        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", msg_ptr->header.stamp.sec, msg_ptr->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "range_range: %.3f - %.3f", msg_ptr->range_min, msg_ptr->range_max);
        RCLCPP_INFO(this->get_logger(), "angle_range: %.3f - %.3f", msg_ptr->angle_min * TO_DEG, msg_ptr->angle_max * TO_DEG);
        RCLCPP_INFO(this->get_logger(), "angle++: %f", msg_ptr->angle_increment * TO_DEG);
        RCLCPP_INFO(this->get_logger(), "time++ : %f", msg_ptr->time_increment);
        RCLCPP_INFO(this->get_logger(), "ranges_size: %ld", msg_ptr->ranges.size());

        for(int i=0, size=msg_ptr->ranges.size(); i<size; i++){
            int x = frame.cols/2 - msg_ptr->ranges[i] * std::sin(msg_ptr->angle_increment * i + msg_ptr->angle_min) / m_resolution;
            int y = frame.rows/2 - msg_ptr->ranges[i] * std::cos(msg_ptr->angle_increment * i + msg_ptr->angle_min) / m_resolution;

            if(x<0 || frame.cols<=x) continue;
            if(y<0 || frame.rows<=y) continue;

            frame.at<cv::Vec3b>(y,x) = this->m_point_color;
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