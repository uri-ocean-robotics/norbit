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
     * @brief Constructor
     */  
    NorbitRos();

    /**
     * @brief Destructor
     */  
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
     * @brief Update the ROS parameters
     */  
    void updateParams();

    /**
     * @brief Setup ROS such as pub/sub/srv/timer...
     */  
    void setupROS();

    /**
     * @brief Setup TCP handler and callbacks
     */ 
    void setupTCP();

    /**
     * @brief Shutdown the sonar (i.e., close sonar power)
     */     
    void closeSonar();

    /**
     * @brief Start the TCP, initialize the sonar
     */      
    void waitForConnections();

    /**
     * @brief Sonar initialization: send param to sonar
     */      
    void initializeSonarParams();
    
    /**
     * @brief ROS service for the CMD send to sonar
     * @param req The request of this service
     * @param resp The response of this service
     * @return True: succ; False: failed
     */  
    bool norbitCmdCallback(
        const std::shared_ptr<norbit_msgs::srv::NorbitCmd::Request> req,
        const std::shared_ptr<norbit_msgs::srv::NorbitCmd::Response> resp);

    /**
     * @brief ROS service for the power setup CMD
     * @param req The request of this service
     * @param resp The response of this service
     * @return True: succ; False: failed
     */  
    bool setPowerCallback(
        const std::shared_ptr<norbit_msgs::srv::SetPower::Request> req,
        const std::shared_ptr<norbit_msgs::srv::SetPower::Response> resp);

    /**
     * @brief Restart the sonar connection
     */  
    void disconnectTimerCallback();

    /**
     * @brief The bathmetry callback from TCP handler
     * @param data The parsed norbit bathymetric data
     */      
    void callbackBathty(norbit_types::BathymetricData data);

    /**
     * @brief The water column callback from TCP handler
     * @param data The parsed norbit water column data
     */      
    void callbackWC(norbit_types::WaterColumnData data);

};

