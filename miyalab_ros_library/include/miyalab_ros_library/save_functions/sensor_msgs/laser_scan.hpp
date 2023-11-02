#ifndef __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_SENSOR_MSGS_LASER_SCAN_HPP__
#define __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_SENSOR_MSGS_LASER_SCAN_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <fstream>

// ROS2
#include <sensor_msgs/msg/laser_scan.hpp>

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{
/**
 * @brief LaserScan save fuction 
 * 
 * @param path save file path
 * @param scan save laser scan
 * @return true
 * @return false
 */
static bool saveLaserScan(const std::string &path, const sensor_msgs::msg::LaserScan &scan)
{
    // empty check
    if(scan.ranges.size() != scan.intensities.size()) return false;

    // file open
    std::ofstream ofs(path);

    // generate header
    char stamp[16];
    std::snprintf(stamp, 16, "%d.%09d", scan.header.stamp.sec, scan.header.stamp.nanosec);
    ofs << "FRAME ID," << scan.header.frame_id << std::endl;
    ofs << "TIME," << stamp << std::endl;
    ofs << "RANGE," << scan.range_min << "," << scan.range_max << std::endl;
    ofs << "ANGLE RANGE," << scan.angle_min << "," << scan.angle_max << std::endl;
    ofs << "ANGLE INCREMENT," << scan.angle_increment << std::endl;
    ofs << "SCAN TIME," << scan.scan_time << std::endl;
    ofs << "TIME INCREMENT," << scan.scan_time << std::endl;
    ofs << "DATA SET:" << std::endl;
    ofs << "RANGE, INTENSITY" << std::endl;

    // save points
    for(int i=0; i<scan.ranges.size(); i++){
        ofs << scan.ranges[i] << "," << scan.intensities[i] << std::endl;
    }

    return true;
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_SENSOR_MSGS_LASER_SCAN_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------