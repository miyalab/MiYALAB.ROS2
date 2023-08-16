#ifndef __MIYALAB_ROS2_DRIVER_MSG_VIEWER_LASER_SCAN_VIEWER_HPP__
#define __MIYALAB_ROS2_DRIVER_MSG_VIEWER_LASER_SCAN_VIEWER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
class LaserScanViewer: public rclcpp::Node {
public:
    LaserScanViewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~LaserScanViewer();
private:
    std::mutex m_mutex;
    sensor_msgs::msg::LaserScan::SharedPtr m_msg_ptr;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_subscriber;
    void onMsgSubscribed(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    double m_resolution = -1;
    double m_range_x = -1, m_range_y = -1;
    cv::Vec3b m_background_color = {0,0,0};
    cv::Vec3b m_point_color = {255,255,255};
    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_LASER_SCAN_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------