#ifndef __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_MIYALAB_INTERFACES_ROBOT_STATE_HPP__
#define __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_MIYALAB_INTERFACES_ROBOT_STATE_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <filesystem>
#include <fstream>

// ROS2
#include <miyalab_interfaces/msg/robot_state.hpp>

// MiYALAB
#include "miyalab_ros_library/type_converter/geometry_msgs/to_rpy.hpp"

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{
/**
 * @brief RobotState save fuction
 * 
 * @param path save file path
 * @param state save robot state
 * @return true
 * @return false
 */
bool saveRobotState(const std::string &path, const miyalab_interfaces::msg::RobotState &state)
{
    // generate header
    if(!std::filesystem::is_regular_file(path)){
        std::ofstream ofs(path);
        ofs << "time,"
            << "battery,"
            << "temp,"
            << "cpu_temp,"
            << "distance,"
            << "pose.x,pose.y,pose.z,"
            << "pose.roll,pose.pitch,pose.yaw,"
            << "imu.ax,imu.ay,imu.az,"
            << "imu.wx,imu.wy,imu.wz,"
            << "imu.ox,imu.oy,imu.oz,";
        for(int i=0, size=state.pwm.size(); i<size; i++){
            ofs << "pwm" << i << ","
                << "encoder" << i << ","
                << "rpm" << i << ","
                << "current" << i << ",";
        }
        ofs << std::endl;
        ofs.close();
    }

    // save robot state
    auto state_odometry_pose_pose_orientation = MiYALAB::ROS2::toRPY(state.odometry.pose.pose.orientation);
    auto state_imu_orientation = MiYALAB::ROS2::toRPY(state.imu.orientation);
    char state_header_stamp_nanosec_nanosec[16];
    std::snprintf(state_header_stamp_nanosec_nanosec, 16, "%09d", state.header.stamp.nanosec);
    std::ofstream ofs(path, std::ios::app);
    ofs << state.header.stamp.sec << "." << state_header_stamp_nanosec_nanosec << ","
        << state.battery << ","
        << state.temperature << ","
        << state.cpu_temperature << ","
        << state.distance << ","
        << state.odometry.pose.pose.position.x << "," << state.odometry.pose.pose.position.y << "," << state.odometry.pose.pose.position.z << ","
        << state_odometry_pose_pose_orientation.x << "," << state_odometry_pose_pose_orientation.y << "," << state_odometry_pose_pose_orientation.z << ","
        << state.imu.linear_acceleration.x << "," << state.imu.linear_acceleration.y << "," << state.imu.linear_acceleration.z << ","
        << state.imu.angular_velocity.x << "," << state.imu.angular_velocity.y << "," << state.imu.angular_velocity.z << ","
        << state_imu_orientation.x << "," << state_imu_orientation.y << "," << state_imu_orientation.z << ",";
    for(int i=0, size=state.pwm.size(); i<size; i++){
        ofs << state.pwm[i] << ","
            << state.encoder[i] << ","
            << state.rpm[i] << ","
            << state.current[i] << ",";
    }
    ofs << std::endl;
    ofs.close();
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_MIYALAB_INTERFACES_ROBOT_STATE_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------