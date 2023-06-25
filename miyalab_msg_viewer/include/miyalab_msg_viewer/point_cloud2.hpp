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
 * @brief Project Name
 * 
 */
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Component Definition
 * 
 */
class PointCloud2Viewer: public rclcpp::Node {
public:
    PointCloud2Viewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    virtual ~PointCloud2Viewer();
private:    
    std::mutex points_mutex;
    sensor_msgs::msg::PointCloud2::SharedPtr points;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscriber;
    void onPointsSubscribed(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    double RESOLUTION = -1;
    double RANGE_X = -1, RANGE_Y = -1;
    cv::Vec3b BACKGROUND_COLOR = {0,0,0};
    cv::Vec3b POINT_COLOR = {255,255,255};
    const int LOOP_RATE = -1;
    template<typename T, typename U>void forceSet(const T *value, const U &set_value){*((T*)value)=set_value;}
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_POINT_CLOUD2_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------