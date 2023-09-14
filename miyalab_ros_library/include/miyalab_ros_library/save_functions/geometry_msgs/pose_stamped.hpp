#ifndef __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_GEOMETRY_MSGS_POSE_STAMPED_HPP__
#define __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_GEOMETRY_MSGS_POSE_STAMPED_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <fstream>

// ROS2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <miyalab_ros_library/type_converter/geometry_msgs/to_rpy.hpp>

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{

static inline bool savePoseStamped(const std::string &path, const geometry_msgs::msg::PoseStamped &pose)
{
    // generate header
    if(!std::filesystem::is_regular_file(path)){
        std::ofstream ofs(path);
        ofs << "time,"
            << "position.x,"
            << "position.y,"
            << "position.z,"
            << "orientation.x,"
            << "orientation.y,"
            << "orientation.z"
            << std::endl;
        ofs.close();
    }

    geometry_msgs::msg::Vector3 pose_pose_orientation;
    MiYALAB::ROS2::toRPY(pose.pose.orientation, &pose_pose_orientation);
    char pose_header_stamp_nanosec[16];
    std::snprintf(pose_header_stamp_nanosec, 16, "%09d", pose.header.stamp.nanosec);
    std::ofstream ofs(path, std::ios::app);
    ofs << pose.header.stamp.sec << "." << pose_header_stamp_nanosec << ","
        << pose.pose.position.x << ","
        << pose.pose.position.y << ","
        << pose.pose.position.z << ","
        << pose_pose_orientation.x << ","
        << pose_pose_orientation.y << ","
        << pose_pose_orientation.z << ","
        << std::endl;

    return true;
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_GEOMETRY_MSGS_POSE_STAMPED_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------