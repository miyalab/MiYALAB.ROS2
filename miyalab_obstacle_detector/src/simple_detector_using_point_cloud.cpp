//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include "miyalab_obstacle_detector/simple_detector_using_point_cloud.hpp"

//-----------------------------
// Namespace & using
//-----------------------------
using geometry_msgs::msg::Vector3;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud;
using miyalab_interfaces::msg::ObstacleArray;

//-----------------------------
// Methods
//-----------------------------
/**
 * @brief Project name
 * 
 */
namespace MiYALAB{
namespace ROS2{

/**
 * @brief Construct a new class object
 * 
 * @param options 
 */
SimpleObstacleDetectorUsingPointCloud::SimpleObstacleDetectorUsingPointCloud(rclcpp::NodeOptions options) : rclcpp::Node("rs_obstacle_detector", options)
{
    // Using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    this->view_range.width  = this->declare_parameter("obstacle_detector.view_range.y", 5.0);
    this->view_range.height = this->declare_parameter("obstacle_detector.view_range.x", 5.0);
    this->detector.setUnit(this->unit = this->declare_parameter("obstacle_detector.unit", 0.01));
    this->detector.setRangeMin(
        this->declare_parameter("obstacle_detector.range_min.x", 0.0),
        this->declare_parameter("obstacle_detector.range_min.y", 0.0),
        this->declare_parameter("obstacle_detector.range_min.z", 0.1)
    );
    this->detector.setRangeMax(
        this->declare_parameter("obstacle_detector.range_max.x", 2.5),
        this->declare_parameter("obstacle_detector.range_max.y", 1.5),
        this->declare_parameter("obstacle_detector.range_max.z", 2.0)
    );
    this->detector.setDilateTimes(this->declare_parameter("obstacle_detector.dilate_times", 5));
    this->detector.setErodeTimes(this->declare_parameter("obstacle_detector.erode_times", 3));
    this->detector.setDilateSquare(this->declare_parameter("obstacle_detector.dilate_square", 5));
    this->detector.setErodeSquare(this->declare_parameter("obstacle_detector.erode_square", 3));
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    this->point_cloud_subscriber    = this->create_subscription<PointCloud>("/lidar/points", 10, std::bind(&SimpleObstacleDetectorUsingPointCloud::onPointCloudSubscribed, this, _1));
    this->detector_range_subscriber = this->create_subscription<Vector3>("/waypoint_manager/obstacle_range", 10, std::bind(&SimpleObstacleDetectorUsingPointCloud::onDetectorRangeSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    this->obstacle_infos_publisher = this->create_publisher<ObstacleArray>("~/obstacles", 10);
    this->obstacle_img_publisher   = this->create_publisher<Image>("~/image", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    // RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Initialize Service-Client 
    // RCLCPP_INFO(this->get_logger(), "Initialize service-clients...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-clients were initialized.");

    // Main loop processing
    this->thread = std::make_unique<std::thread>(&SimpleObstacleDetectorUsingPointCloud::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
SimpleObstacleDetectorUsingPointCloud::~SimpleObstacleDetectorUsingPointCloud()
{
    this->thread.release();
}

void SimpleObstacleDetectorUsingPointCloud::onPointCloudSubscribed(const PointCloud::SharedPtr points)
{
    if(points->points.empty()) return;
    this->point_cloud_mutex.lock();
    this->point_cloud = points;
    this->point_cloud_mutex.unlock();
}


void SimpleObstacleDetectorUsingPointCloud::onDetectorRangeSubscribed(const Vector3::SharedPtr range)
{
    this->detector_mutex.lock();
    this->detector.setRangeMin(this->detector.getRangeMin().x, this->detector.getRangeMin().y, range->z);
    this->detector.setRangeMax(range->x, range->y, this->detector.getRangeMax().z);
    this->detector_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void SimpleObstacleDetectorUsingPointCloud::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());

    // Main loop
    for(rclcpp::WallRate loop(30); rclcpp::ok(); loop.sleep()){
        this->point_cloud_mutex.lock();
        auto points = this->point_cloud;
        this->point_cloud = nullptr;
        this->point_cloud_mutex.unlock();

        if(points.get()){
            // ---------------------------- 障害物検知main -------------------------------
            cv_bridge::CvImage debug_img(points->header, "bgr8", cv::Mat(2 * this->view_range.height/this->unit + 1, 2 * this->view_range.width/this->unit + 1, CV_8UC3, cv::Scalar(0,0,0)));
            cv::Point center(debug_img.image.cols/2, debug_img.image.rows/2);
            cv::rectangle(debug_img.image, 
                cv::Rect(
                    cv::Point(center.x - this->detector.getRangeMax().y/this->unit, center.y - this->detector.getRangeMax().x/this->unit), 
                    cv::Point(center.x + this->detector.getRangeMax().y/this->unit, center.y - this->detector.getRangeMin().x/this->unit)), 
                cv::Scalar(0,255,255),
                2);
            cv::circle(debug_img.image, center, 5, cv::Scalar(255,255,0), -1);
            cv::circle(debug_img.image, center, debug_img.image.cols*1/8, cv::Scalar(255,255,0), 1);
            cv::circle(debug_img.image, center, debug_img.image.cols*2/8, cv::Scalar(255,255,0), 1);
            cv::circle(debug_img.image, center, debug_img.image.cols*3/8, cv::Scalar(255,255,0), 1);
            cv::circle(debug_img.image, center, debug_img.image.cols*4/8, cv::Scalar(255,255,0), 1);
            cv::rectangle(debug_img.image,
                cv::Rect(center.x-0.3/this->unit, center.y-0.2/this->unit, 0.6/this->unit, 0.78/this->unit),
                cv::Scalar(255,0,255),
                1
            );
            for(const auto &point: points->points){
                if(point.x < -this->view_range.height || this->view_range.height < point.x) continue;
                if(point.y < -this->view_range.width || this->view_range.width < point.y) continue;
                if(point.z < this->detector.getRangeMin().z || this->detector.getRangeMax().z < point.z) continue;
                debug_img.image.at<cv::Vec3b>(center.y - point.x/this->unit, center.x - point.y/this->unit) = cv::Vec3b(255, 255, 255);
            }

            cv_bridge::CvImage detect_img(points->header, "bgr8");
            auto obstacles_msg = std::make_unique<ObstacleArray>();
            auto obstacles_img_msg = std::make_unique<Image>();
            
            this->detector_mutex.lock();
            this->detector.detect(*points, obstacles_msg.get(), &detect_img.image);
            this->detector_mutex.unlock();
            for(int i=0, size=obstacles_msg->data.size(); i<size; i++){
                const cv::Point point(center.x - obstacles_msg->data[i].centroid.y/this->unit, center.y - obstacles_msg->data[i].centroid.x/this->unit);
                cv::circle(debug_img.image, point, 3, cv::Scalar(0,0,255), -1);
                
            }
            char text[256];
            std::snprintf(text, sizeof(text), "obstacle_cnt: %ld", obstacles_msg->data.size());
            cv::putText(debug_img.image, text, cv::Point(10, debug_img.image.rows - 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,255), 2);
            std::snprintf(text, sizeof(text), "range_min: (%.2lf, %.2lf, %.2lf)", this->detector.getRangeMin().x, this->detector.getRangeMin().y, this->detector.getRangeMin().z);
            cv::putText(debug_img.image, text, cv::Point(10, debug_img.image.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,255), 2);
            std::snprintf(text, sizeof(text), "range_max: (%.2lf, %.2lf, %.2lf)", this->detector.getRangeMax().x, this->detector.getRangeMax().y, this->detector.getRangeMax().z);
            cv::putText(debug_img.image, text, cv::Point(10, debug_img.image.rows - 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,255), 2);
            
            // detect_img.toImageMsg(*obstacle_img_msg);
            debug_img.toImageMsg(*obstacles_img_msg);
            
            this->obstacle_infos_publisher->publish(std::move(obstacles_msg));
            this->obstacle_img_publisher->publish(std::move(obstacles_img_msg));
        }
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::SimpleObstacleDetectorUsingPointCloud)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------