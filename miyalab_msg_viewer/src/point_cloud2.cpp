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
PointCloud2Viewer::PointCloud2Viewer(rclcpp::NodeOptions options) : rclcpp::Node("point_cloud2_viewer", options)
{
    // Using placeholders
    using std::placeholders::_1;
    
    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_resolution = this->declare_parameter("point_cloud2_viewer.resolution", 0.5);
    m_range_x    = this->declare_parameter("point_cloud2_viewer.range.x", 10.0);
    m_range_y    = this->declare_parameter("point_cloud2_viewer.range.y", 10.0);
    m_background_color[0] = this->declare_parameter("point_cloud2_viewer.background_color.b", 0);
    m_background_color[1] = this->declare_parameter("point_cloud2_viewer.background_color.g", 0);
    m_background_color[2] = this->declare_parameter("point_cloud2_viewer.background_color.r", 0);
    m_point_color[0] = this->declare_parameter("point_cloud2_viewer.point_color.b", 255);
    m_point_color[1] = this->declare_parameter("point_cloud2_viewer.point_color.g", 255);
    m_point_color[2] = this->declare_parameter("point_cloud2_viewer.point_color.r", 255);
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_subscriber = this->create_subscription<PointCloud2>("/lidar/points", 10, std::bind(&PointCloud2Viewer::onMsgSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&PointCloud2Viewer::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
PointCloud2Viewer::~PointCloud2Viewer()
{
    m_thread.release();
}

void PointCloud2Viewer::onMsgSubscribed(const PointCloud2::SharedPtr msg)
{
    m_mutex.lock();
    m_msg_ptr = msg;
    m_mutex.unlock();
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
        m_mutex.lock();
        auto points_ptr = m_msg_ptr;
        m_msg_ptr = nullptr;
        m_mutex.unlock();
        if(!points_ptr.get()) continue;

        cv::Size frame_size(2*(m_range_x/m_resolution)+1, 2*(m_range_y/m_resolution)+1);
        cv::Mat frame(frame_size, CV_8UC3, m_background_color);
        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", points_ptr->header.stamp.sec, points_ptr->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "bigendian: %s", points_ptr->is_bigendian ? "YES" : "NO");
        for(int i=0, size=points_ptr->fields.size(); i<size; i++){
            RCLCPP_INFO(this->get_logger(), "field[%d]: %s", i, points_ptr->fields[i].name.c_str());
            RCLCPP_INFO(this->get_logger(), "datatype: %s", DATATYPE_STR[points_ptr->fields[i].datatype]);
            RCLCPP_INFO(this->get_logger(), "offset  : %d", points_ptr->fields[i].offset);
            RCLCPP_INFO(this->get_logger(), "count   : %d", points_ptr->fields[i].count);
        }

        cv::imshow(m_subscriber->get_topic_name(), frame);
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