#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <marine_acoustic_msgs/msg/sonar_detections.hpp>
#include <marine_acoustic_msgs/msg/sonar_ranges.hpp>
#include <marine_acoustic_msgs/msg/raw_sonar_image.hpp>

#include <norbit_msgs/msg/bathymetric_stamped.hpp>
#include <norbit_msgs/msg/water_column_stamped.hpp>
#include <norbit_msgs/srv/norbit_cmd.hpp>
#include <norbit_msgs/srv/set_power.hpp>

#include <norbit/default.hpp>
#include <norbit/parameters.hpp>
#include <norbit/tcp_socket_handler.hpp>
#include <norbit/ros_helper.hpp>

class NorbitRos : public rclcpp::Node
{
public:
    /**
     * @brief something here
     */  
    NorbitRos();

    ~NorbitRos();

private:
    // ===================================================================== //
    // ROS variables
    // ===================================================================== // 

    //! @brief PointCloud2 publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr 
        cloud_pub_;

    //! @brief Bathymetric publisher
    rclcpp::Publisher<norbit_msgs::msg::BathymetricStamped>::SharedPtr 
        bathy_pub_;

    //! @brief Detections publisher
    rclcpp::Publisher<marine_acoustic_msgs::msg::SonarDetections>::SharedPtr 
        detect_pub_;

    //! @brief Range publisher
    rclcpp::Publisher<marine_acoustic_msgs::msg::SonarRanges>::SharedPtr 
        ranges_pub_;

    //! @brief Watercolumn publisher
    rclcpp::Publisher<norbit_msgs::msg::WaterColumnStamped>::SharedPtr 
        norbit_wc_pub_;

    //! @brief Watercolumn raw image publisher
    rclcpp::Publisher<marine_acoustic_msgs::msg::RawSonarImage>::SharedPtr 
        wc_pub_;

    //! @brief Norbit CMD service
    rclcpp::Service<norbit_msgs::srv::NorbitCmd>::SharedPtr
        norbit_cmd_srv_;

    //! @brief Norbit set power service
    rclcpp::Service<norbit_msgs::srv::SetPower>::SharedPtr
        set_power_srv_;

    //! @brief Timer callback to disconnect the sensor
    rclcpp::TimerBase::SharedPtr disconnect_timer_;

    // ===================================================================== //
    // other variables
    // ===================================================================== // 

    //! @brief parameters for the sensor
    ConnectionParams params_;

    //! UDP handler object pointer
    std::shared_ptr<TCPSocketHandler> tcp_handler_;

    // ===================================================================== //
    // Functions
    // ===================================================================== //  

    /**
     * @brief something here
     */  
    void updateParams();

    /**
     * @brief something here
     */  
    void setupPubSub();

    /**
     * @brief something here
     */ 
    void setupTCP();

    void closeSonar();

    void waitForConnections();

    void initializeSonarParams();
    
    /**
     * @brief something here
     */  
    bool norbitCmdCallback(
        const std::shared_ptr<norbit_msgs::srv::NorbitCmd::Request> req,
        const std::shared_ptr<norbit_msgs::srv::NorbitCmd::Response> resp);

    /**
     * @brief something here
     */  
    bool setPowerCallback(
        const std::shared_ptr<norbit_msgs::srv::SetPower::Request> req,
        const std::shared_ptr<norbit_msgs::srv::SetPower::Response> resp);

    /**
     * @brief something here
     */  
    void disconnectTimerCallback();

    void callbackBathty(norbit_types::BathymetricData data);

    void callbackWC(norbit_types::WaterColumnData data);

};

