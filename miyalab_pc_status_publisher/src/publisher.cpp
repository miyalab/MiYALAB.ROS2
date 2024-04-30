//-----------------------------
// include
//-----------------------------
// STL
#include <fstream>
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "miyalab_pc_status_publisher/publisher.hpp"

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
PCStatusPublisher::PCStatusPublisher(rclcpp::NodeOptions options) : rclcpp::Node("PC", options)
{
    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_rate        = this->declare_parameter("pc_status_publisher.rate", 1);
    m_frame_id    = this->declare_parameter("pc_status_publisher.frame_id", "pc_status");
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    m_pc_status_publisher = this->create_publisher<miyalab_interfaces::msg::PCStatus>("~/status", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&PCStatusPublisher::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
PCStatusPublisher::~PCStatusPublisher()
{
    m_thread.release();
}

/**
 * @brief Execute method
 * 
 */
void PCStatusPublisher::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(m_rate); rclcpp::ok(); loop.sleep()){
        auto msg = std::make_unique<miyalab_interfaces::msg::PCStatus>();

        msg->header.frame_id = m_frame_id;
        msg->header.stamp = this->now();

        // CPU周波数
        auto ifs = std::ifstream("/proc/cpuinfo");
        if(ifs){
            std::string line = "";
            while(std::getline(ifs, line)){
                float freq = 0.0;
                std::sscanf(line.c_str(), "cpu MHz :%f", &freq);
                if(freq != 0.0){
                    msg->cpu_freq.emplace_back(freq);
                }
            }
            // std::string line = "";
            // std::getline(ifs, line);
            // std::sscanf(line.c_str(), "%f", &msg->cpu_freq);
            ifs.close();
        }

        // CPU温度
        ifs = std::ifstream("/sys/class/thermal/thermal_zone0/temp");
        if(ifs){
            std::string line = "";
            std::getline(ifs, line);
            std::sscanf(line.c_str(), "%f", &msg->cpu_temperature);
            msg->cpu_temperature /= 1000;
            ifs.close();
        }

        // メモリ使用率
        ifs = std::ifstream("/proc/meminfo");
        if(ifs){
            float total = 1.0;
            float free = 1.0;
            std::string line = "";
            std::getline(ifs, line);
            std::sscanf(line.c_str(), "MemTotal:%fkB", &total);
            std::getline(ifs, line);
            std::getline(ifs, line);
            std::sscanf(line.c_str(), "MemAvailable:%fkB", &free);
            msg->memory_usage = 1.0 - (free / total);
            ifs.close();
        }

        // publish
        m_pc_status_publisher->publish(std::move(msg));
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::PCStatusPublisher)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------