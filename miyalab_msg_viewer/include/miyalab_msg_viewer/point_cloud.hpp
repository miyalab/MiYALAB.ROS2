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
 * @brief Project Name
 * 
 */
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Component Definition
 * 
 */
class PointCloudViewer: public rclcpp::Node {
public:
    PointCloudViewer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    virtual ~PointCloudViewer();
private:    
    std::mutex points_mutex;
    sensor_msgs::msg::PointCloud::SharedPtr points;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr points_subscriber;
    void onPointsSubscribed(const sensor_msgs::msg::PointCloud::SharedPtr msg);
    
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

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_POINT_CLOUD_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------