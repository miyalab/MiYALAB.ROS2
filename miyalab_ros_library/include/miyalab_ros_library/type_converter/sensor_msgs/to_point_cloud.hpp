#ifndef __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_SENSOR_MSGS_TO_POINT_CLOUD_HPP__
#define __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_SENSOR_MSGS_TO_POINT_CLOUD_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <cmath>

// ROS2
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Convert LaserScan to PointCloud 
 * 
 * @param laser 
 * @return sensor_msgs::msg::PointCloud 
 */
inline sensor_msgs::msg::PointCloud toPointCloud(const sensor_msgs::msg::LaserScan &laser)
{
    sensor_msgs::msg::PointCloud ret;
    int points_size = laser.ranges.size();
    ret.points.resize(points_size);
    ret.channels.resize(2);
    ret.channels[0].name = "range";
    ret.channels[1].name = "intensity";
    ret.channels[0].values.resize(points_size);
    ret.channels[1].values.resize(points_size);
    
    for(int i=0; i<points_size; i++){
        double angle = laser.angle_increment * i + laser.angle_min;
        ret.points[i].x = laser.ranges[i] * std::cos(angle);
        ret.points[i].y = laser.ranges[i] * std::sin(angle);
        ret.points[i].z = 0;
        ret.channels[0].values[i] = laser.ranges[i];
        ret.channels[1].values[i] = laser.intensities[i];
    }
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_SENSOR_MSGS_TO_POINT_CLOUD_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------