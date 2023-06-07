#ifndef __MIYALAB_ROS2_DRIVER_MSG_VIEWER_HPP__
#define __MIYALAB_ROS2_DRIVER_MSG_VIEWER_HPP__

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
    std::mutex laser_mutex;
    sensor_msgs::msg::LaserScan::SharedPtr laser;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber;
    void onLaserScanSubscribed(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    double RESOLUTION = -1;
    double RANGE_X = -1, RANGE_Y = -1;
    template<typename T, typename U>void forceSet(const T *value, const U &set){*((T*)value)=set;}
    std::unique_ptr<std::thread> thread;
    void run();
};
}
}

#endif // __MIYALAB_ROS2_DRIVER_MSG_VIEWER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------