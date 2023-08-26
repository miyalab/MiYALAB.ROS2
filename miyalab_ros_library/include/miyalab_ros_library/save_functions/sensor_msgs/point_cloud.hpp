#ifndef __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_SENSOR_MSGS_POINT_CLOUD_HPP__
#define __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_SENSOR_MSGS_POINT_CLOUD_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <fstream>

// ROS2
#include <sensor_msgs/msg/point_cloud.hpp>

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{
/**
 * @brief PointCloud save fuction using ascii
 * 
 * @param path save file path
 * @param points save points
 * @return true
 * @return false
 */
static bool savePointCloudAscii(const std::string &path, const sensor_msgs::msg::PointCloud &points)
{
    // empty check
    if(points.points.empty()) return false;

    // file open
    std::ofstream ofs(path);
    int points_size = points.points.size();
    int channel_size = points.channels.size();

    // generate header
    ofs << "# .PCD v.7 - Point Cloud Data file format" << std::endl
        << "VERSION .7" << std::endl
        << "FIELDS x y z";
    for(int i=0; i<channel_size; i++) ofs << " " << points.channels[i].name;
    ofs << std::endl 
        << "SIZE 4 4 4" << std::endl;
    for(int i=0; i<channel_size; i++) ofs << " 4";
    ofs << std::endl 
        << "COUNT 1 1 1" << std::endl;
    for(int i=0; i<channel_size; i++) ofs << " 1";
    ofs << std::endl 
        << "WIDTH " << points_size << std::endl
        << "HEIGHT 1" << std::endl
        << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
        << "POINTS " << points_size << std::endl
        << "DATA ascii" << std::endl;

    // save points
    for(int i=0; i<points_size; i++){
        ofs << points.points[i].x << " " << points.points[i].y << " " << points.points[i].z;
        for(int j=0; j<channel_size; j++) ofs << " " << points.channels[j].values[i];
        ofs << std::endl;
    }

    return true;
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_SENSOR_MSGS_POINT_CLOUD_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------