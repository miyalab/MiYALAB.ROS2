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
 * @param roll 
 * @param pitch 
 * @param yaw 
 * @return geometry_msgs::msg::Quaternion 
 */
static inline geometry_msgs::msg::Quaternion toQuaternion(const double &roll, const double &pitch, const double &yaw)
{
    geometry_msgs::msg::Quaternion ret;
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    ret.w = cr * cp * cy + sr * sp * sy;
    ret.x = sr * cp * cy - cr * sp * sy;
    ret.y = cr * sp * cy + sr * cp * sy;
    ret.z = cr * cp * sy - sr * sp * cy;
    return ret;
}

/**
 * @brief Convert Roll-Pitch-Yaw angle type To Quaternion
 * 
 * @param rpy 
 * @return geometry_msgs::msg::Quaternion 
 */
static inline geometry_msgs::msg::Quaternion toQuaternion(const geometry_msgs::msg::Vector3 &rpy)
{
    return MiYALAB::ROS2::toQuaternion(rpy.x, rpy.y, rpy.z);
}

/**
 * @brief Convert Roll-Pitch-Yaw angle type To Quaternion
 * 
 * @param roll          input
 * @param pitch         input
 * @param yaw           input
 * @param quaternion    output
 */
static inline void toQuaternion(const double &roll, const double &pitch, const double &yaw, geometry_msgs::msg::Quaternion *quaternion)
{
    *quaternion = MiYALAB::ROS2::toQuaternion(roll, pitch, yaw);
}

/**
 * @brief Convert Roll-Pitch-Yaw angle type To Quaternion
 * 
 * @param rpy        input
 * @param quaternion output
 */
static inline void toQuaternion(const geometry_msgs::msg::Vector3 &rpy, geometry_msgs::msg::Quaternion *quaternion)
{
    *quaternion = MiYALAB::ROS2::toQuaternion(rpy);
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_GEOMETRY_MSGS_TO_QUATERNION_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------