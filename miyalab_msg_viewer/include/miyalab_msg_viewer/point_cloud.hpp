#ifndef __MIYALAB_ROS2_DRIVER_MSG_VIEWER_POINT_CLOUD_VIEWER_HPP__
#define __MIYALAB_ROS2_DRIVER_MSG_VIEWER_POINT_CLOUD_VIEWER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <thread>
#include <memory>
#include <mutex>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

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
 * @brief PointCloud viewer
 * 
 */
class PointCloudViewer: public rclcpp::Node {
public:
    PointCloudViewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    virtual ~PointCloudViewer();
private:    
    std::mutex m_mutex;
    sensor_msgs::msg::PointCloud::SharedPtr m_msg_ptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr m_subscriber;
    void onMsgSubscribed(const sensor_msgs::msg::PointCloud::SharedPtr msg);
    
    struct Param{
        double resolution;
        double range_x, range_y;
        cv::Vec3b background_color;
        cv::Vec3b point_color;
        bool chart_enable;
        int chart_range_increment;
        double chart_angle_increment;
        cv::Vec3b chart_color;
    } m_param;
    std::unique_ptr<std::thread> m_thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_POINT_CLOUD_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------