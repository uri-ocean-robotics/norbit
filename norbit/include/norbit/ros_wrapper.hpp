#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class NorbitRos : public rclcpp::Node
{
public:
    NorbitRos();

private:
    void setupParam();

    void setupSubpub();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

};