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

#include "miyalab_msg_viewer/point_cloud.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using sensor_msgs::msg::PointCloud;

//-----------------------------
// const value
//-----------------------------
constexpr double TO_DEG = 180 / M_PI;

//-----------------------------
// Methods
//-----------------------------
/**
 * @brief MiYALAB ROS2
 * 
 */
namespace MiYALAB{
namespace ROS2{
/**
 * @brief Construct a new class object
 * 
 * @param options 
 */
PointCloudViewer::PointCloudViewer(rclcpp::NodeOptions options) : rclcpp::Node("point_cloud_viewer", options)
{
    // Using placeholders
    using std::placeholders::_1;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_resolution = this->declare_parameter("point_cloud_viewer.resolution", 0.5);
    m_range_x    = this->declare_parameter("point_cloud_viewer.range.x", 10.0);
    m_range_y    = this->declare_parameter("point_cloud_viewer.range.y", 10.0);
    m_background_color[0] = this->declare_parameter("point_cloud_viewer.background_color.b", 0);
    m_background_color[1] = this->declare_parameter("point_cloud_viewer.background_color.g", 0);
    m_background_color[2] = this->declare_parameter("point_cloud_viewer.background_color.r", 0);
    m_point_color[0] = this->declare_parameter("point_cloud_viewer.point_color.b", 255);
    m_point_color[1] = this->declare_parameter("point_cloud_viewer.point_color.g", 255);
    m_point_color[2] = this->declare_parameter("point_cloud_viewer.point_color.r", 255);
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_subscriber = this->create_subscription<PointCloud>("/lidar/points", 10, std::bind(&PointCloudViewer::onMsgSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&PointCloudViewer::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
PointCloudViewer::~PointCloudViewer()
{
    m_thread.release();
}

void PointCloudViewer::onMsgSubscribed(const PointCloud::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "subscribed");
    m_mutex.lock();
    m_msg_ptr = msg;
    m_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void PointCloudViewer::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        m_mutex.lock();
        auto points_ptr = m_msg_ptr;
        m_msg_ptr = nullptr;
        m_mutex.unlock();
        if(!points_ptr.get()) continue;

        cv::Size frame_size(2*(m_range_x/m_resolution)+1, 2*(m_range_y/m_resolution)+1);
        cv::Mat frame(frame_size, CV_8UC3, m_background_color);
        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", points_ptr->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", points_ptr->header.stamp.sec, points_ptr->header.stamp.nanosec);
        for(int i=0, size=points_ptr->channels.size(); i<size; i++){
            RCLCPP_INFO(this->get_logger(), "channel[%d]: %s", i, points_ptr->channels[i].name.c_str());
        }

        for(int i=0, size=points_ptr->points.size(); i<size; i++){
            int x = frame.cols/2 - points_ptr->points[i].y/m_resolution;
            int y = frame.cols/2 - points_ptr->points[i].x/m_resolution;

            if(x<0 || frame.cols<=x) continue;
            if(y<0 || frame.rows<=y) continue;

            frame.at<cv::Vec3b>(y,x) = m_point_color;
        }

        cv::imshow("frame", frame);
        cv::waitKey(1);
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::PointCloudViewer)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------