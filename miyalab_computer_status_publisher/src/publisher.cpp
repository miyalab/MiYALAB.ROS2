//-----------------------------
// include
//-----------------------------
// STL
#include <fstream>
#include <memory>
#include <thread>
#include <functional>
#include <filesystem>

// linux
#include <sys/times.h>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include "miyalab_computer_status_publisher/publisher.hpp"

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
ComputerStatusPublisher::ComputerStatusPublisher(rclcpp::NodeOptions options) : rclcpp::Node("computer", options)
{
    // Using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    m_rate        = this->declare_parameter("pc_status_publisher.rate", 1);
    m_frame_id    = this->declare_parameter("pc_status_publisher.frame_id", "computer_status");
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    m_computer_status_publisher = this->create_publisher<miyalab_interfaces::msg::ComputerStatus>("~/status", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize service
    RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    m_computer_info_server = this->create_service<miyalab_interfaces::srv::GetComputerInfo>("~/get_computer_info", std::bind(&ComputerStatusPublisher::serviceGetComputerInfo, this, _1, _2, _3));
    RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Initialize computer info
    readComputerInfo();

    // Main loop processing
    m_thread = std::make_unique<std::thread>(&ComputerStatusPublisher::run, this);
    m_thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
ComputerStatusPublisher::~ComputerStatusPublisher()
{
    m_thread.release();
}

void ComputerStatusPublisher::readComputerInfo()
{
    m_computer_info_response = std::make_shared<miyalab_interfaces::srv::GetComputerInfo::Response>();

    // CPUコア数取得
    while(std::filesystem::exists("/sys/devices/system/cpu/cpu" + std::to_string(m_computer_info_response->cpu_count_cores++)));
    m_computer_info_response->cpu_count_cores -= 1;

    // CPU最大周波数取得
    m_computer_info_response->cpu_freq_max.resize(m_computer_info_response->cpu_count_cores);
    for(int i=0; i<m_computer_info_response->cpu_count_cores; i++){
        auto ifs = std::ifstream("/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_max_freq");
        if(ifs){
            std::string line;
            std::getline(ifs, line);
            std::sscanf(line.c_str(), "%lf", &m_computer_info_response->cpu_freq_max[i]);
            m_computer_info_response->cpu_freq_max[i] /= 1000.0;
            ifs.close();
        }
        else{
            m_computer_info_response->cpu_freq_max[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }

    // CPU最小周波数取得
    m_computer_info_response->cpu_freq_min.resize(m_computer_info_response->cpu_count_cores);
    for(int i=0; i<m_computer_info_response->cpu_count_cores; i++){    
        auto ifs = std::ifstream("/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_min_freq");
        if(ifs){
            std::string line;
            std::getline(ifs, line);
            std::sscanf(line.c_str(), "%lf", &m_computer_info_response->cpu_freq_min[i]);
            m_computer_info_response->cpu_freq_min[i] /= 1000.0;
            ifs.close();
        }
        else{
            m_computer_info_response->cpu_freq_min[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }

    // CPU現在周波数ファイル有無チェック
    m_class_cpu_file_exists = true;
    for(int i=0; i<m_computer_info_response->cpu_count_cores && m_class_cpu_file_exists; i++){
        m_class_cpu_file_exists = std::filesystem::exists("/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_cur_freq");
    }

    // メモリ情報
    auto ifs = std::ifstream("/proc/meminfo");
    m_computer_info_response->memory_size = std::numeric_limits<double>::quiet_NaN();
    if(ifs){
        std::string line;
        while(std::getline(ifs, line)){
            if(line.find("MemTotal:") != std::string::npos){
                std::sscanf(line.c_str(), "MemTotal: %lf", &m_computer_info_response->memory_size);
                m_computer_info_response->memory_size /= 1024.0;
            }
        }
        ifs.close();
    }
    
    // DEBUG
    RCLCPP_INFO(this->get_logger(), "count cores: %ld", m_computer_info_response->cpu_count_cores);
    for(int i=0; i<m_computer_info_response->cpu_count_cores; i++){
        RCLCPP_INFO(this->get_logger(), "core %d: (%.0lf MHz, %.0lf MHz)", 
            i, 
            m_computer_info_response->cpu_freq_min[i], 
            m_computer_info_response->cpu_freq_max[i]
        );
    }
    RCLCPP_INFO(this->get_logger(), "memory size: %.0lf MB", m_computer_info_response->memory_size);
}

void ComputerStatusPublisher::serviceGetComputerInfo(const std::shared_ptr<rmw_request_id_t> header, 
                            const miyalab_interfaces::srv::GetComputerInfo::Request::SharedPtr request,
                            const miyalab_interfaces::srv::GetComputerInfo::Response::SharedPtr response)
{
    m_computer_info_mutex.lock();
    response->cpu_count_cores = m_computer_info_response->cpu_count_cores;
    response->cpu_freq_min = m_computer_info_response->cpu_freq_min;
    response->cpu_freq_max = m_computer_info_response->cpu_freq_max;
    response->memory_size = m_computer_info_response->memory_size;
    m_computer_info_mutex.unlock();
    response->success = true;
}

/**
 * @brief Execute method
 * 
 */
void ComputerStatusPublisher::run()
{
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " has started. thread id = " << std::this_thread::get_id());

    m_computer_info_mutex.lock();
    auto cpu_count_cores = m_computer_info_response->cpu_count_cores;
    auto memory_size = m_computer_info_response->memory_size;
    m_computer_info_mutex.unlock();
    
    // Main loop
    long pre_tick = 0;
    clock_t pre_time = times(NULL);
    for(rclcpp::WallRate loop(m_rate); rclcpp::ok(); loop.sleep()){
        auto msg = std::make_unique<miyalab_interfaces::msg::ComputerStatus>();

        msg->header.frame_id = m_frame_id;
        msg->header.stamp = this->now();

        // CPU周波数
        msg->cpu_freq.resize(cpu_count_cores);
        if(m_class_cpu_file_exists){
            for(int i=0; i<cpu_count_cores; i++){
                std::string line;
                auto ifs = std::ifstream("/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_cur_freq");
                std::getline(ifs, line);
                ifs.close();
                sscanf(line.c_str(), "%lf", &msg->cpu_freq[i]);
                msg->cpu_freq[i] /= 1000.0;
            }
        }
        else{
            auto ifs = std::ifstream("/proc/cpuinfo");
            if(ifs){
                std::string line = "";
                int processor_id = -1;
                double freq = 0.0;
                while(std::getline(ifs, line)){
                    std::sscanf(line.c_str(), "processor : %d", &processor_id);
                    if(std::sscanf(line.c_str(), "cpu MHz : %lf", &freq) == 1) msg->cpu_freq[processor_id] = freq;
                }
                ifs.close();
            }
        }

        // CPU温度
        for(int i=0; std::filesystem::exists("/sys/class/hwmon/hwmon" + std::to_string(i)); i++){
            std::string dev_name;
            auto ifs = std::ifstream("/sys/class/hwmon/hwmon" + std::to_string(i) + "/name");
            std::getline(ifs, dev_name);
            ifs.close();
            for(int j=1; ifs = std::ifstream("/sys/class/hwmon/hwmon" + std::to_string(i) + "/temp" + std::to_string(j) + "_input"); j++){
                sensor_msgs::msg::Temperature temp;
                temp.header.frame_id = dev_name;
                temp.header.stamp = msg->header.stamp;

                std::string line;
                std::getline(ifs, line);
                ifs.close();
                sscanf(line.c_str(), "%lf", &temp.temperature);
                temp.temperature /= 1000.0;

                ifs = std::ifstream("/sys/class/hwmon/hwmon" + std::to_string(i) + "/temp" + std::to_string(j) + "_label");
                if(std::getline(ifs, line)) temp.header.frame_id += " " + line;
                ifs.close();

                msg->temperatures.emplace_back(temp);
            }
        }

        // メモリ使用率
        auto ifs = std::ifstream("/proc/meminfo");
        msg->memory_usage = std::numeric_limits<double>::quiet_NaN();
        if(ifs){
            double free = 1.0;
            for(std::string line = ""; std::sscanf(line.c_str(), "MemAvailable: %lf kB", &free) != 1; std::getline(ifs, line));
            ifs.close();
            msg->memory_usage = 1.0 - (free/1024.0 / memory_size);
        }

        // CPU使用率
        ifs = std::ifstream("/proc/stat");
        if(ifs){
            std::string line;
            std::getline(ifs, line);
            char name[128];
            long usr, nice, sys;
            sscanf(line.c_str(), "%s %ld %ld %ld", name, &usr, &nice, &sys);
            long tick = usr + nice + sys;
            clock_t time_now = times(NULL);
            msg->cpu_usage = (double)(tick - pre_tick) / (time_now - pre_time) / cpu_count_cores;

            pre_tick = tick;
            pre_time = time_now;
        }
        else msg->cpu_usage = std::numeric_limits<double>::quiet_NaN();

        // publish
        m_computer_status_publisher->publish(std::move(msg));
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiYALAB::ROS2::ComputerStatusPublisher)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------