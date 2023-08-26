#ifndef __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_RPY_HPP__
#define __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_RPY_HPP__

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
 * @brief Convert Quaternion to Roll-Pitch-Yaw angle type
 * 
 * @param quaternion 
 * @return geometry_msgs::msg::Vector3 
 */
inline geometry_msgs::msg::Vector3 toRPY(const geometry_msgs::msg::Quaternion &quaternion)
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
}
}

#endif // __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_RPY_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------