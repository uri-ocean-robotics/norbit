#include <norbit/ros_wrapper.hpp>

NorbitRos::NorbitRos()
    : Node("norbit_ros_node")
{

    //! Load param
    setupParam();

}

void NorbitRos::setupParam()
{
    //! IP address
    std::string ip;
    this->declare_parameter<std::string>("ip", "127.0.0.1");
    if (this->get_parameter("ip", ip)){
        RCLCPP_INFO(this->get_logger(), "ip: %s", ip.c_str());
    }
    else {
        //! if set default, this not need anymore
        RCLCPP_ERROR(this->get_logger(), "ip: no param available!");
    }

}

void NorbitRos::setupSubpub()
{
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 20);
}