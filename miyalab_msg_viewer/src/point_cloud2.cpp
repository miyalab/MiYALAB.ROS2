//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// MiYALAB
#include <miyalab_ros_library/type_converter/sensor_msgs/to_point_cloud.hpp>

#include "miyalab_msg_viewer/point_cloud2.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using sensor_msgs::msg::PointCloud;
using sensor_msgs::msg::PointCloud2;

//-----------------------------
// const value
//-----------------------------
constexpr double TO_DEG = 180.0 / M_PI;
constexpr double TO_RAD = M_PI / 180.0;
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
    m_param.resolution = this->declare_parameter("point_cloud2_viewer.resolution", 0.5);
    m_param.range_x    = this->declare_parameter("point_cloud2_viewer.range.x", 10.0);
    m_param.range_y    = this->declare_parameter("point_cloud2_viewer.range.y", 10.0);
    m_param.background_color[0] = this->declare_parameter("point_cloud2_viewer.background_color.b", 0);
    m_param.background_color[1] = this->declare_parameter("point_cloud2_viewer.background_color.g", 0);
    m_param.background_color[2] = this->declare_parameter("point_cloud2_viewer.background_color.r", 0);
    m_param.point_color[0] = this->declare_parameter("point_cloud2_viewer.point_color.b", 255);
    m_param.point_color[1] = this->declare_parameter("point_cloud2_viewer.point_color.g", 255);
    m_param.point_color[2] = this->declare_parameter("point_cloud2_viewer.point_color.r", 255);
    m_param.chart_enable = this->declare_parameter("point_cloud2_viewer.chart.enable", true);
    m_param.chart_range_increment  = this->declare_parameter("point_cloud2_viewer.chart.range", 1.0) / m_param.resolution;
    m_param.chart_angle_increment  = this->declare_parameter("point_cloud2_viewer.chart.angle", 45.0) * TO_RAD;
    m_param.chart_color[0] = this->declare_parameter("point_cloud2_viewer.chart_color.b", 255);
    m_param.chart_color[1] = this->declare_parameter("point_cloud2_viewer.chart_color.g", 0);
    m_param.chart_color[2] = this->declare_parameter("point_cloud2_viewer.chart_color.r", 0);
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

    double range_max = std::hypot(m_param.range_x, m_param.range_y) / m_param.resolution;
    cv::Size frame_size(2*(m_param.range_x/m_param.resolution)+1, 2*(m_param.range_y/m_param.resolution)+1);
    cv::Point center_point(frame_size.width/2, frame_size.height/2);
    
    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        m_mutex.lock();
        auto points_ptr = m_msg_ptr;
        m_msg_ptr = nullptr;
        m_mutex.unlock();
        if(!points_ptr.get()) continue;

        RCLCPP_INFO(this->get_logger(), "---");

        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", points_ptr->header.stamp.sec, points_ptr->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", points_ptr->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "bigendian: %s", points_ptr->is_bigendian ? "YES" : "NO");
        for(int i=0, size=points_ptr->fields.size(); i<size; i++){
            RCLCPP_INFO(this->get_logger(), "field[%d]: %s", i, points_ptr->fields[i].name.c_str());
            RCLCPP_INFO(this->get_logger(), "--datatype: %s", DATATYPE_STR[points_ptr->fields[i].datatype]);
            RCLCPP_INFO(this->get_logger(), "--offset  : %d", points_ptr->fields[i].offset);
            RCLCPP_INFO(this->get_logger(), "--count   : %d", points_ptr->fields[i].count);
        }

        cv::Mat frame(frame_size, CV_8UC3, m_param.background_color);
        if(m_param.chart_enable){
            for(int range = m_param.chart_range_increment; range < range_max; range += m_param.chart_range_increment){
                cv::circle(frame, center_point, range, m_param.chart_color, 1);
            }
            for(double angle=0; angle<2*M_PI; angle+=m_param.chart_angle_increment){
                cv::line(frame,
                    center_point,
                    cv::Point((range_max * std::cos(angle)) + center_point.x, (range_max * std::sin(angle)) + center_point.y),
                    m_param.chart_color, 1);
            }
        }

        PointCloud points;
        if(MiYALAB::ROS2::toPointCloud(*points_ptr, &points)){
           for(int i=0, size=points.points.size(); i<size; i++){
                int x = frame.cols/2 - points.points[i].y/m_param.resolution;
                int y = frame.rows/2 - points.points[i].x/m_param.resolution;

                if(x<0 || frame.cols<=x) continue;
                if(y<0 || frame.rows<=y) continue;

                frame.at<cv::Vec3b>(y,x) = m_param.point_color;
            }
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