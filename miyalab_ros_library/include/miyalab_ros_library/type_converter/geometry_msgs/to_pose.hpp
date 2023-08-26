#ifndef __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_POSE_HPP__
#define __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_POSE_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <cmath>

// ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{
inline geometry_msgs::msg::Pose toPose(const geometry_msgs::msg::Pose2D &pose2d)
{
    geometry_msgs::msg::Pose ret;
    ret.position.x = pose2d.x;
    ret.position.y = pose2d.y;
    ret.position.z = 0;
    ret.orientation.w = std::cos(0.5 * pose2d.theta);
    ret.orientation.x = 0;
    ret.orientation.y = 0;
    ret.orientation.z = std::sin(0.5 * pose2d.theta);
    return ret;
}

}
}

#endif // __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_POSE_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------