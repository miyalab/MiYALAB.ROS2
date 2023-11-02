#ifndef __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_GEOMETRY_MSGS_POSE_HPP__
#define __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_GEOMETRY_MSGS_POSE_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <fstream>
#include <filesystem>

// ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <miyalab_ros_library/type_converter/geometry_msgs/to_rpy.hpp>

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{

static inline bool savePoseStamped(const std::string &path, const geometry_msgs::msg::Pose &pose)
{
    // generate header
    if(!std::filesystem::is_regular_file(path)){
        std::ofstream ofs(path);
        ofs << "position.x,"
            << "position.y,"
            << "position.z,"
            << "orientation.x,"
            << "orientation.y,"
            << "orientation.z"
            << std::endl;
        ofs.close();
    }

    geometry_msgs::msg::Vector3 pose_orientation;
    MiYALAB::ROS2::toRPY(pose.orientation, &pose_orientation);
    std::ofstream ofs(path, std::ios::app);
    ofs << pose.position.x << ","
        << pose.position.y << ","
        << pose.position.z << ","
        << pose_orientation.x << ","
        << pose_orientation.y << ","
        << pose_orientation.z
        << std::endl;

    return true;
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_GEOMETRY_MSGS_POSE_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------