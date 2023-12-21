#ifndef __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_SENSOR_MSGS_TO_POINT_CLOUD_HPP__
#define __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_SENSOR_MSGS_TO_POINT_CLOUD_HPP__

//-----------------------------
// include
//-----------------------------
// STL
#include <cmath>

// ROS2
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

//-----------------------------
// Functions
//-----------------------------
namespace MiYALAB {
namespace ROS2{
/**
 * @brief Convert LaserScan to PointCloud 
 * 
 * @param laser 
 * @return sensor_msgs::msg::PointCloud 
 */
static inline sensor_msgs::msg::PointCloud toPointCloud(const sensor_msgs::msg::LaserScan &laser)
{
    sensor_msgs::msg::PointCloud ret;
    int points_size = laser.ranges.size();
    ret.points.resize(points_size);
    ret.channels.resize(2);
    ret.channels[0].name = "range";
    ret.channels[1].name = "intensity";
    ret.channels[0].values.resize(points_size);
    ret.channels[1].values.resize(points_size);
    
    for(int i=0; i<points_size; i++){
        double angle = laser.angle_increment * i + laser.angle_min;
        ret.points[i].x = laser.ranges[i] * std::cos(angle);
        ret.points[i].y = laser.ranges[i] * std::sin(angle);
        ret.points[i].z = 0;
        ret.channels[0].values[i] = laser.ranges[i];
        ret.channels[1].values[i] = laser.intensities[i];
    }

    return ret;
}

/**
 * @brief Convert LaserScan to PointCloud 
 * 
 * @param laser  input
 * @param points output
 */
static inline void toPointCloud(const sensor_msgs::msg::LaserScan &laser, sensor_msgs::msg::PointCloud *points)
{
    *points = MiYALAB::ROS2::toPointCloud(laser);
}

/**
 * @brief Convert PointCloud2 to PointCloud
 * 
 * @param input  PointCloud2 type value
 * @param output PointCloud type value
 * @return true 
 * @return false 
 */
static inline bool toPointCloud(const sensor_msgs::msg::PointCloud2 &input, sensor_msgs::msg::PointCloud *output)
{
    constexpr int DATA_SIZE[] = {0,1,1,2,2,4,4,4,8};

    if(input.data.empty()) return false;

    int points_size = input.width * input.height;
    int fields_size  = input.fields.size();
    int channels_size = fields_size - 3;

    output->header = input.header;
    output->points.resize(points_size);
    output->channels.resize(channels_size);
    for(int i=0; i<channels_size; i++) output->channels[i].values.resize(points_size);
    
    unsigned char xyz_check = 0;
    std::vector<int> fields_channel(fields_size);
    for(int i=0, channel_index=0; i<fields_size; i++){
        if(input.fields[i].name == "x")      xyz_check |= 0x01;
        else if(input.fields[i].name == "y") xyz_check |= 0x02;
        else if(input.fields[i].name == "z") xyz_check |= 0x04;
        else{
            fields_channel[i] = channel_index;
            output->channels[channel_index].name = input.fields[i].name;
            channel_index++;
        }
    }
    if(xyz_check != 0x07) return false;
    for(int i=0; i<points_size; i++){
        for(int j=0; j<fields_size; j++){
            // 格納先
            float *value;
            if(input.fields[j].name == "x")      value = &output->points[i].x;
            else if(input.fields[j].name == "y") value = &output->points[i].y;
            else if(input.fields[j].name == "z") value = &output->points[i].z;
            else value = &output->channels[fields_channel[j]].values[i];
            
            // データ格納
            uint8_t *data_ptr;
            int index = input.point_step * i + input.fields[j].offset;
            if((input.is_bigendian && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__) || (!input.is_bigendian && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)){
                data_ptr = (uint8_t*)&input.data[index];
            }
            else{
                uint8_t data[8];
                index += DATA_SIZE[input.fields[j].datatype] - 1;
                for(int k=0; k<DATA_SIZE[input.fields[j].datatype]; k++) data[k] = input.data[index - k];
                data_ptr = data;
            }

            if(input.fields[j].datatype == sensor_msgs::msg::PointField::FLOAT64)      *value = *(double*)(data_ptr);
            else if(input.fields[j].datatype == sensor_msgs::msg::PointField::FLOAT32) *value = *(float*)(data_ptr);
            else if(input.fields[j].datatype == sensor_msgs::msg::PointField::UINT32)  *value = *(uint32_t*)(data_ptr);
            else if(input.fields[j].datatype == sensor_msgs::msg::PointField::INT32)   *value = *(int32_t*)(data_ptr);
            else if(input.fields[j].datatype == sensor_msgs::msg::PointField::UINT16)  *value = *(uint16_t*)(data_ptr);
            else if(input.fields[j].datatype == sensor_msgs::msg::PointField::INT16)   *value = *(int16_t*)(data_ptr);
            else if(input.fields[j].datatype == sensor_msgs::msg::PointField::UINT8)   *value = *(uint8_t*)(data_ptr);
            else if(input.fields[j].datatype == sensor_msgs::msg::PointField::INT8)    *value = *(int8_t*)(data_ptr);
        }
    }

    return true;
}
}
}

#endif // __MIYALAB_ROS2_LIBRARY_TYPE_CONVERTER_SENSOR_MSGS_TO_POINT_CLOUD_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------