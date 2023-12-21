#ifndef __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_SENSOR_MSGS_POINT_CLOUD2_HPP__
#define __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_SENSOR_MSGS_POINT_CLOUD2_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <fstream>

// ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>

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
static bool savePointCloudAscii(const std::string &path, const sensor_msgs::msg::PointCloud2 &points)
{
    constexpr int DATA_SIZE[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};
    constexpr char DATA_TYPE[] = " IUIUIUFF";

    // empty check
    if(points.points.empty()) return false;

    // file open
    std::ofstream ofs(path);
    int points_size = points.height * points.width;
    int fields_size = points.fields.size();

    // generate header
    ofs << "# .PCD v.7 - Point Cloud Data file format" << std::endl
        << "VERSION .7" << std::endl
        << "FIELDS";
    for(int i=0; i<fields_size; i++) ofs << " " << points.fields[i].name;
    ofs << std::endl 
        << "SIZE";
    for(int i=0; i<fields_size; i++) ofs << " " << DATA_SIZE[points.fields[i].datatype];
    ofs << std::endl
        << "TYPE";
    for(int i=0; i<fields_size; i++) ofs << " " << DATA_TYPE[points.fields[i].datatype];
    ofs << std::endl 
        << "COUNT";
    for(int i=0; i<channel_size; i++) ofs << " " << points.fields[i].count;
    ofs << std::endl 
        << "WIDTH " << points_size << std::endl
        << "HEIGHT 1" << std::endl
        << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
        << "POINTS " << points_size << std::endl
        << "DATA ascii" << std::endl;

    // save points
    for(int i=0; i<points_size; i++){
        for(int j=0; j<fields_size; j++){
    
        }
    }

    return true;
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_SAVE_FUNCTIONS_SENSOR_MSGS_POINT_CLOUD_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------