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
    if(points.data.empty()) return false;

    // file open
    std::ofstream ofs(path);
    int points_size = points.height * points.width;
    int fields_size = points.fields.size();

    // generate header
    ofs << "# .PCD v.7 - Point Cloud Data file format" << std::endl
        << "VERSION .7" << std::endl
        << "FIELDS";
    for(int i=0; i<fields_size; i++) ofs << " " + points.fields[i].name;
    ofs << std::endl 
        << "SIZE";
    for(int i=0; i<fields_size; i++) ofs << " " << DATA_SIZE[points.fields[i].datatype];
    ofs << std::endl
        << "TYPE";
    for(int i=0; i<fields_size; i++) ofs << " " << DATA_TYPE[points.fields[i].datatype];
    ofs << std::endl 
        << "COUNT";
    for(int i=0; i<fields_size; i++) ofs << " " << points.fields[i].count;
    ofs << std::endl 
        << "WIDTH " << points_size << std::endl
        << "HEIGHT 1" << std::endl
        << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
        << "POINTS " << points_size << std::endl
        << "DATA ascii" << std::endl;

    // save points
    for(int i=0; i<points_size; i++){
        for(int j=0; j<fields_size; j++){
            uint8_t *data_ptr;
            int index = points.point_step * i + points.fields[j].offset;
            if((points.is_bigendian && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__) || (!points.is_bigendian && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)){
                data_ptr = (uint8_t*)(&data_ptr[index]);
            }
            else{
                uint8_t data[8];
                index += DATA_SIZE[points.fields[j].datatype] - 1;
                for(int k=0; k<DATA_SIZE[points.fields[j].datatype]; k++) data[k] = points.data[index - k];
                data_ptr = data;
            }

            if(points.fields[j].datatype == sensor_msgs::msg::PointField::FLOAT64)      ofs << *(double*)(data_ptr);
            else if(points.fields[j].datatype == sensor_msgs::msg::PointField::FLOAT32) ofs << *(float*)(data_ptr);
            else if(points.fields[j].datatype == sensor_msgs::msg::PointField::UINT32)  ofs << *(uint32_t*)(data_ptr);
            else if(points.fields[j].datatype == sensor_msgs::msg::PointField::INT32)   ofs << *(int32_t*)(data_ptr);
            else if(points.fields[j].datatype == sensor_msgs::msg::PointField::UINT16)  ofs << *(uint16_t*)(data_ptr);
            else if(points.fields[j].datatype == sensor_msgs::msg::PointField::INT16)   ofs << *(int16_t*)(data_ptr);
            else if(points.fields[j].datatype == sensor_msgs::msg::PointField::UINT8)   ofs << *(uint8_t*)(data_ptr);
            else if(points.fields[j].datatype == sensor_msgs::msg::PointField::INT8)    ofs << *(int8_t*)(data_ptr);
            ofs << " ";
        }
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