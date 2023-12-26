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

#include "miyalab_msg_viewer/laser_scan.hpp"

//-----------------------------
// namespace & using
//-----------------------------
using sensor_msgs::msg::LaserScan;

//-----------------------------
// const value
//-----------------------------
constexpr double TO_DEG = 180 / M_PI;
constexpr double TO_RAD = M_PI / 180.0;

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
LaserScanViewer::LaserScanViewer(rclcpp::NodeOptions options) : rclcpp::Node("laser_scan_viewer", options)
{
    // Using placeholders
    using std::placeholders::_1;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_param.resolution = this->declare_parameter("laser_scan_viewer.resolution", 0.5);
    m_param.range_x    = this->declare_parameter("laser_scan_viewer.range.x", 10.0);
    m_param.range_y    = this->declare_parameter("laser_scan_viewer.range.y", 10.0);
    m_param.background_color[0] = this->declare_parameter("laser_scan_viewer.background_color.b", 0);
    m_param.background_color[1] = this->declare_parameter("laser_scan_viewer.background_color.g", 0);
    m_param.background_color[2] = this->declare_parameter("laser_scan_viewer.background_color.r", 0);
    m_param.point_color[0] = this->declare_parameter("laser_scan_viewer.point_color.b", 255);
    m_param.point_color[1] = this->declare_parameter("laser_scan_viewer.point_color.g", 255);
    m_param.point_color[2] = this->declare_parameter("laser_scan_viewer.point_color.r", 255);
    m_param.chart_enable = this->declare_parameter("laser_scan_viewer.chart.enable", true);
    m_param.chart_range_increment  = this->declare_parameter("laser_scan_viewer.chart.range", 1.0) / m_param.resolution;
    m_param.chart_angle_increment  = this->declare_parameter("laser_scan_viewer.chart.angle", 45.0) * TO_RAD;
    m_param.chart_color[0] = this->declare_parameter("laser_scan_viewer.chart_color.b", 255);
    m_param.chart_color[1] = this->declare_parameter("laser_scan_viewer.chart_color.g", 0);
    m_param.chart_color[2] = this->declare_parameter("laser_scan_viewer.chart_color.r", 0);
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    m_subscriber = this->create_subscription<LaserScan>("/laser/scan", 10, std::bind(&LaserScanViewer::onMsgSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&LaserScanViewer::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
LaserScanViewer::~LaserScanViewer()
{
    m_thread.release();
}

void LaserScanViewer::onMsgSubscribed(const LaserScan::SharedPtr msg)
{
    m_mutex.lock();
    m_msg_ptr = msg;
    m_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void LaserScanViewer::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());

    double range_max = std::hypot(m_param.range_x, m_param.range_y) / m_param.resolution;
    cv::Size frame_size(2*(m_param.range_x/m_param.resolution)+1, 2*(m_param.range_y/m_param.resolution)+1);
    cv::Point center_point(frame_size.width/2, frame_size.height/2);

    // Main loop
    for(rclcpp::WallRate loop(10); rclcpp::ok(); loop.sleep()){
        m_mutex.lock();
        auto msg_ptr = m_msg_ptr;
        m_msg_ptr = nullptr;
        m_mutex.unlock();
        if(!msg_ptr.get()) continue;
        
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

        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "stamp: %d.%09d", msg_ptr->header.stamp.sec, msg_ptr->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", msg_ptr->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "range_range: %.3f - %.3f", msg_ptr->range_min, msg_ptr->range_max);
        RCLCPP_INFO(this->get_logger(), "angle_range: %.3f - %.3f", msg_ptr->angle_min * TO_DEG, msg_ptr->angle_max * TO_DEG);
        RCLCPP_INFO(this->get_logger(), "angle++: %f", msg_ptr->angle_increment * TO_DEG);
        RCLCPP_INFO(this->get_logger(), "time++ : %f", msg_ptr->time_increment);
        RCLCPP_INFO(this->get_logger(), "ranges_size: %ld", msg_ptr->ranges.size());

        for(int i=0, size=msg_ptr->ranges.size(); i<size; i++){
            double angle = msg_ptr->angle_increment * i + msg_ptr->angle_min;
            int x = frame.cols/2 - msg_ptr->ranges[i] * std::sin(angle) / m_param.resolution;
            int y = frame.rows/2 - msg_ptr->ranges[i] * std::cos(angle) / m_param.resolution;

            if(x<0 || frame.cols<=x) continue;
            if(y<0 || frame.rows<=y) continue;

            frame.at<cv::Vec3b>(y,x) = this->m_param.point_color;
        }

        cv::imshow(m_subscriber->get_topic_name(), frame);
        cv::waitKey(1);
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::LaserScanViewer)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------