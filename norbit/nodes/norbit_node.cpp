#include <rclcpp/rclcpp.hpp>
#include <norbit/ros_wrapper.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NorbitRos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}