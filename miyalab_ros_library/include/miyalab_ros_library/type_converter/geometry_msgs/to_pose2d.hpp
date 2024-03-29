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
/**
 * @brief Convert Pose to Pose2D
 * 
 * @param pose   input
 * @param pose2d output
 */
static inline void toPose2D(const geometry_msgs::msg::Pose &pose, geometry_msgs::msg::Pose2D *pose2d)
{
    pose2d->x = pose.position.x;
    pose2d->y = pose.position.y;
    pose2d->theta = MiYALAB::ROS2::toRPY(pose.orientation).z;
}

/**
 * @brief Convert Pose to Pose2D
 * 
 * @param pose 
 * @return geometry_msgs::msg::Pose2D 
 */
static inline geometry_msgs::msg::Pose2D toPose2D(const geometry_msgs::msg::Pose &pose)
{
    geometry_msgs::msg::Pose2D ret;
    MiYALAB::ROS2::toPose2D(pose, &ret);
    return ret;
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_POSE_2D_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------