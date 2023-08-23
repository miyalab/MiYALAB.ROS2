#ifndef __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_HPP__
#define __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <cmath>

// ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// Class
//-----------------------------
/**
 * @brief MiYALAB ROS2 Library
 * 
 */
namespace MiYALAB {
namespace ROS2{
inline geometry_msgs::msg::Quaternion toQuaternion(const geometry_msgs::msg::Vector3 &vector)
{
    geometry_msgs::msg::Quaternion ret;
    const double cy = std::cos(vector.z * 0.5);
    const double sy = std::sin(vector.z * 0.5);
    const double cp = std::cos(vector.y * 0.5);
    const double sp = std::sin(vector.y * 0.5);
    const double cr = std::cos(vector.x * 0.5);
    const double sr = std::sin(vector.x * 0.5);
    ret.w = cr * cp * cy + sr * sp * sy;
    ret.x = sr * cp * cy - cr * sp * sy;
    ret.y = cr * sp * cy + sr * cp * sy;
    ret.z = cr * cp * sy - sr * sp * cy;
    return ret;
}

inline geometry_msgs::msg::Vector3 toVector(const geometry_msgs::msg::Quaternion &quaternion)
{
    geometry_msgs::msg::Vector3 ret;
    const double q0q0 = quaternion.w * quaternion.w;
    const double q1q1 = quaternion.x * quaternion.x;
    const double q2q2 = quaternion.y * quaternion.y;
    const double q3q3 = quaternion.z * quaternion.z;
    const double q0q1 = quaternion.w * quaternion.x;
    const double q0q2 = quaternion.w * quaternion.y;
    const double q0q3 = quaternion.w * quaternion.z;
    const double q1q2 = quaternion.x * quaternion.y;
    const double q1q3 = quaternion.x * quaternion.z;
    const double q2q3 = quaternion.y * quaternion.z;
    ret.x = std::atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    ret.y = -std::asin(2.0 * (q1q3 - q0q2));
    ret.z = std::atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
    return ret;
}

inline geometry_msgs::msg::Pose2D toPose2D(const geometry_msgs::msg::Pose &pose)
{
    geometry_msgs::msg::Pose2D ret;
    ret.x = pose.position.x;
    ret.y = pose.position.y;
    ret.theta = toVector(pose.orientation).z;
    return ret;
}

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

#endif // __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------