//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

#include "miyalab_msg_viewer/point_cloud2.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using sensor_msgs::msg::PointCloud2;

//-----------------------------
// const value
//-----------------------------
constexpr double TO_DEG = 180 / M_PI;
constexpr char DATATYPE_STR[][8] = {
    "",
    "INT8",
    "UINT8",
    "INT16",
    "UINT16",
    "INT32",
    "UINT32",
    "FLOAT32",
    "FLOAT64"
};

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
PointCloud2Viewer::PointCloud2Viewer(rclcpp::NodeOptions options) : rclcpp::Node("point_cloud_viewer", options)
{
    // Using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    this->forceSet(&this->RESOLUTION, this->declare_parameter("point_cloud_viewer.resolution", 0.5));
    this->forceSet(&this->RANGE_X, this->declare_parameter("point_cloud_viewer.range.x", 10.0));
    this->forceSet(&this->RANGE_Y, this->declare_parameter("point_cloud_viewer.range.y", 10.0));
    this->forceSet(&this->BACKGROUND_COLOR[0], this->declare_parameter("point_cloud_viewer.background_color.b", 0));
    this->forceSet(&this->BACKGROUND_COLOR[1], this->declare_parameter("point_cloud_viewer.background_color.g", 0));
    this->forceSet(&this->BACKGROUND_COLOR[2], this->declare_parameter("point_cloud_viewer.background_color.r", 0));
    this->forceSet(&this->POINT_COLOR[0], this->declare_parameter("point_cloud_viewer.point_color.b", 255));
    this->forceSet(&this->POINT_COLOR[1], this->declare_parameter("point_cloud_viewer.point_color.g", 255));
    this->forceSet(&this->POINT_COLOR[2], this->declare_parameter("point_cloud_viewer.point_color.r", 255));
    this->points = std::make_shared<PointCloud2>();
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    this->points_subscriber = this->create_subscription<PointCloud2>("/lidar/points", 10, std::bind(&PointCloud2Viewer::onPointsSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    // RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    // RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Initialize Service-Client 
    // RCLCPP_INFO(this->get_logger(), "Initialize service-clients...");
    // RCLCPP_INFO(this->get_logger(), "Complete! Service-clients were initialized.");

    // Main loop processing
    this->thread = std::make_unique<std::thread>(&PointCloud2Viewer::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
PointCloud2Viewer::~PointCloud2Viewer()
{
    this->thread.release();
}

void PointCloud2Viewer::onPointsSubscribed(const PointCloud2::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "subscribed");
    this->points_mutex.lock();
    this->points = msg;
    this->points_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void PointCloud2Viewer::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        this->points_mutex.lock();
        auto points_ptr = this->points;
        this->points_mutex.unlock();

        cv::Size frame_size(2*(this->RANGE_X/this->RESOLUTION)+1, 2*(this->RANGE_Y/this->RESOLUTION)+1);
        cv::Mat frame(frame_size, CV_8UC3, this->BACKGROUND_COLOR);
        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", points_ptr->header.stamp.sec, points_ptr->header.stamp.nanosec);
        for(int i=0, size=points_ptr->fields.size(); i<size; i++){
            RCLCPP_INFO(this->get_logger(), "field[%d]: %s", i, points_ptr->fields[i].name.c_str());
            RCLCPP_INFO(this->get_logger(), "datatype: %s", DATATYPE_STR[points_ptr->fields[i].datatype]);
        }

        // for(int i=0, size=points_ptr->points.size(); i<size; i++){
        //     int x = frame.cols/2 - points_ptr->points[i].y/this->RESOLUTION;
        //     int y = frame.cols/2 - points_ptr->points[i].x/this->RESOLUTION;

        //     if(x<0 || frame.cols<=x) continue;
        //     if(y<0 || frame.rows<=y) continue;

        //     frame.at<cv::Vec3b>(y,x) = this->POINT_COLOR;
        // }

        cv::imshow("frame", frame);
        cv::waitKey(1);
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::PointCloud2Viewer)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------