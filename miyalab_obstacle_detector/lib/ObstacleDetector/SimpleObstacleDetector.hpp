#ifndef __MIYALAB_CPP_ROBOTICS_OBSTACLE_DETECTOR_SIMPLE_OBSTACLE_DETECTOR_HPP__
#define __MIYALAB_CPP_ROBOTICS_OBSTACLE_DETECTOR_SIMPLE_OBSTACLE_DETECTOR_HPP__

//-----------------------------
// include
//-----------------------------
#include <chrono>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include "miyalab_interfaces/msg/obstacle_array.hpp"

//-----------------------------
// Namespace & using
//-----------------------------
using geometry_msgs::msg::Vector3;
using geometry_msgs::msg::Pose;
using sensor_msgs::msg::PointCloud;
using miyalab_interfaces::msg::Obstacle;
using miyalab_interfaces::msg::ObstacleArray;

//-----------------------------
// Class
//-----------------------------
/**
 * @brief Project Ryusei
 * 
 */
namespace MiYALAB {
namespace Robotics{
class SimpleObstacleDetector {
public:
    SimpleObstacleDetector(){}
    virtual ~SimpleObstacleDetector(){}

    void setUnit(const double &unit){m_unit = unit;}
    void setRangeMin(const cv::Point3d &range){this->range_min = range;}
    void setRangeMin(const double &x, const double &y, const double &z){this->range_min.x = x; this->range_min.y = y; this->range_min.z = z;}
    void setRangeMax(const cv::Point3d &range){this->range_max = range;}
    void setRangeMax(const double &x, const double &y, const double &z){this->range_max.x = x; this->range_max.y = y; this->range_max.z = z;}
    void setDilateTimes(const int &times){this->dilate_times = times;}
    void setErodeTimes(const int &times){this->erode_times = times;}
    void setDilateSquare(const int &dilate){this->dilate_square = dilate;}
    void setErodeSquare(const int &erode){this->erode_square = erode;}

    const cv::Point3d &getRangeMin() const {return this->range_min;}
    const cv::Point3d &getRangeMax() const {return this->range_max;}

    void detect(const PointCloud &points, ObstacleArray *obstacles, cv::Mat *dst);

private:
    double m_unit;
    cv::Point3d range_max;
    cv::Point3d range_min;
    int dilate_square;
    int erode_square;
    int dilate_times;
    int erode_times;
    cv::Size img_size;

    void detectLineObject(const Pose &pose, ObstacleArray &obstacles, Obstacle &result);
    void detectRectObject(const Pose &pose, ObstacleArray &obstacles, Obstacle &result);
};
}
}

#endif // __MIYALAB_CPP_ROBOTICS_OBSTACLE_DETECTOR_SIMPLE_OBSTACLE_DETECTOR_HPP__

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------