#ifndef __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_POSE_2D_HPP__
#define __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_POSE_2D_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <cmath>

// ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "to_rpy.hpp"

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{
inline geometry_msgs::msg::Pose2D toPose2D(const geometry_msgs::msg::Pose &pose)
{
    geometry_msgs::msg::Pose2D ret;
    ret.x = pose.position.x;
    ret.y = pose.position.y;
    ret.theta = MiYALAB::ROS2::(pose.orientation).z;
    return ret;
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_POSE_2D_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------