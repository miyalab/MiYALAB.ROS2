

//-----------------------------
// include
//-----------------------------
// STL
#include <iostream>
#include <memory>
#include <thread>
#include <functional>
#include <cmath>
#include "SimpleObstacleDetector.hpp"

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// Methods
//-----------------------------
/**
 * @brief Project name
 * 
 */
namespace MiYALAB{
namespace Robotics{
void SimpleObstacleDetector::detect(const PointCloud &points, ObstacleArray *obstacles, cv::Mat *dst)
{
    const cv::Size img_size(this->range_max.y * 2 / m_unit + 1, this->range_max.x * 2 / m_unit + 1);
    *dst = cv::Mat(img_size, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat binary_img(img_size, CV_8UC1, cv::Scalar(0));

    for(size_t i=0, size=points.points.size(); i<size; i++){
        if(points.points[i].x <  this->range_min.x || this->range_max.x < points.points[i].x) continue;
        if(points.points[i].y < -this->range_max.y || this->range_max.y < points.points[i].y) continue;
        if(points.points[i].z <  this->range_min.z || this->range_max.z < points.points[i].z) continue;

        const cv::Point point(
            img_size.width /2 - points.points[i].y/m_unit,
            img_size.height/2 - points.points[i].x/m_unit
        );
        dst->at<cv::Vec3b>(point) = cv::Vec3b(255,255,255);
        binary_img.at<unsigned char>(point) = 255;
    }

    cv::Mat label, stats, centroids;
    
    cv::dilate(binary_img, binary_img, cv::Mat(), cv::Point(-1, -1), this->dilate_square);
    cv::erode(binary_img, binary_img, cv::Mat(), cv::Point(-1, -1), this->erode_square);

    int label_cnt = cv::connectedComponentsWithStats(binary_img, label, stats, centroids);

    for(int i=1; i<label_cnt; i++){
        Obstacle obs;
        int *param = stats.ptr<int>(i);
        cv::rectangle(*dst, 
            cv::Rect(param[cv::ConnectedComponentsTypes::CC_STAT_LEFT],
                     param[cv::ConnectedComponentsTypes::CC_STAT_TOP],
                     param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH],
                     param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT]
            ),
            cv::Scalar(0,255,0),
            2
        );

        double x = (double)(binary_img.rows / 2 - param[cv::ConnectedComponentsTypes::CC_STAT_TOP]) * m_unit;
        double y = (double)(binary_img.cols / 2 - param[cv::ConnectedComponentsTypes::CC_STAT_LEFT]) * m_unit;
        double width = (double)param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH] * m_unit;
        double height = (double)param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT] * m_unit;

        obs.index = i;
        obs.centroid.x = x - height / 2;
        obs.centroid.y = y - width / 2;
        obs.shape.points.resize(4);
        obs.shape.points[0].x = x;
        obs.shape.points[0].y = y;
        obs.shape.points[1].x = x + width;
        obs.shape.points[1].y = y;
        obs.shape.points[2].x = x + width;
        obs.shape.points[2].y = y + height;
        obs.shape.points[3].x = x;
        obs.shape.points[3].y = y + height;

        cv::circle(*dst, cv::Point(binary_img.cols/2 - obs.centroid.y/m_unit, binary_img.rows/2 - obs.centroid.x/m_unit), 3, cv::Scalar(0,0,255), -1);

        obstacles->data.push_back(obs);
    }

    cv::circle(*dst, cv::Point(dst->cols/2, dst->rows/2), 5, cv::Scalar(255,0,0), -1);
    cv::circle(*dst, cv::Point(dst->cols/2, dst->rows/2), dst->cols/8, cv::Scalar(255,0,0), 1);
    cv::circle(*dst, cv::Point(dst->cols/2, dst->rows/2), dst->cols*2/8, cv::Scalar(255,0,0), 1);
    cv::circle(*dst, cv::Point(dst->cols/2, dst->rows/2), dst->cols*3/8, cv::Scalar(255,0,0), 1);
    cv::circle(*dst, cv::Point(dst->cols/2, dst->rows/2), dst->cols*4/8, cv::Scalar(255,0,0), 1);
}
}
}

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------