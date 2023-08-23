#ifndef __MIYALAB_ROS2_DRIVER_MSG_VIEWER_POINT_CLOUD2_VIEWER_HPP__
#define __MIYALAB_ROS2_DRIVER_MSG_VIEWER_POINT_CLOUD2_VIEWER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

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
 * @brief PointCloud2 viewer
 * 
 */
class PointCloud2Viewer: public rclcpp::Node {
public:
    PointCloud2Viewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    virtual ~PointCloud2Viewer();
private:    
    std::mutex m_mutex;
    sensor_msgs::msg::PointCloud2::SharedPtr m_msg_ptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscriber;
    void onMsgSubscribed(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    double m_resolution = -1;
    double m_range_x = -1, m_range_y = -1;
    cv::Vec3b m_background_color = {0,0,0};
    cv::Vec3b m_point_color = {255,255,255};
    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_POINT_CLOUD2_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------