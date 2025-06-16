#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <norbit/default.hpp>
#include <norbit/parameters.hpp>

class NorbitRos : public rclcpp::Node
{
public:
    NorbitRos();

private:
    // ===================================================================== //
    // ROS variables
    // ===================================================================== // 

    //! PointCloud2 publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr 
        pc_pub_;

    // ===================================================================== //
    // other variables
    // ===================================================================== // 

    ConnectionParams params_;

    // ===================================================================== //
    // Functions
    // ===================================================================== //  

    void updateParams();

    void setupSubpub();

};

