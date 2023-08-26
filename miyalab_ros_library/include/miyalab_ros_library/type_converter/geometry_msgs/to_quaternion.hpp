#ifndef __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_QUATERNION_HPP__
#define __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_QUATERNION_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <cmath>

// ROS2
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Convert Roll-Pitch-Yaw angle type To Quaternion
 * 
 * @param rpy 
 * @return geometry_msgs::msg::Quaternion 
 */
inline geometry_msgs::msg::Quaternion toQuaternion(const geometry_msgs::msg::Vector3 &rpy)
{
    geometry_msgs::msg::Quaternion ret;
    const double cy = std::cos(rpy.z * 0.5);
    const double sy = std::sin(rpy.z * 0.5);
    const double cp = std::cos(rpy.y * 0.5);
    const double sp = std::sin(rpy.y * 0.5);
    const double cr = std::cos(rpy.x * 0.5);
    const double sr = std::sin(rpy.x * 0.5);
    ret.w = cr * cp * cy + sr * sp * sy;
    ret.x = sr * cp * cy - cr * sp * sy;
    ret.y = cr * sp * cy + sr * cp * sy;
    ret.z = cr * cp * sy - sr * sp * cy;
    return ret;
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_QUATERNION_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------