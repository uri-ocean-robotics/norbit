#ifndef NORBIT_CONNECTION_H
#define NORBIT_CONNECTION_H

#include <atomic>
#include <boost/algorithm/string.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <deque>
#include <future>
#include <iostream>
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "norbit_msgs/NorbitCmd.h"
#include "norbit_msgs/SetPower.h"
#include "norbit_types/bathymetric_data.h"
#include "norbit_types/header.h"
#include "norbit_types/message.h"

// using namespace boost::asio;
// using ip::tcp;

struct ConnectionParams {
  std::string ip;
  int bathy_port;
  int cmd_port;
  std::string sensor_frame;
  std::string pointcloud_topic;
  std::string bathymetric_topic;
  double cmd_timeout;
  std::map<std::string, std::string> startup_settings;
  std::map<std::string, std::string> shutdown_settings;
};

class NorbitConnection {
public:
  NorbitConnection();
  ~NorbitConnection();
  void updateParams();
  void setupPubSub();
  void setupConnections();
  norbit_msgs::CmdResp sendCmd(std::string const &cmd, const std::string &val);
  void listenForCmd();
  void receiveCmd(const boost::system::error_code &err);
  void receive();
  void
  recHandler(const boost::system::error_code &error, // Result of operation.
             std::size_t bytes_transferred // Number of bytes received.
  );

  // norbit TCP callbacks
  void bathyCallback(norbit_types::BathymetricData data);

  // ROS callbacks
  bool norbitCmdCallback(norbit_msgs::NorbitCmd::Request &req,
                         norbit_msgs::NorbitCmd::Response &resp);

  bool setPowerCallback(norbit_msgs::SetPower::Request &req,
                        norbit_msgs::SetPower::Response &resp);

  // operations
  void spin();

protected:
  struct {
    std::unique_ptr<boost::asio::ip::tcp::socket> bathymetric;
    std::unique_ptr<boost::asio::ip::tcp::socket> cmd;
  } sockets_;
  //  struct {
  //    ros::ServiceServer norbit_cmd;
  //  } srv_;
  std::map<std::string, ros::ServiceServer> srv_map_;
  boost::asio::io_service io_service_;
  boost::array<char, sizeof(norbit_types::Header)> recv_buffer_;
  boost::array<char, 50000> dataBuffer_;
  boost::asio::streambuf cmd_resp_buffer_;
  ConnectionParams params_;
  ros::NodeHandle node_;
  ros::NodeHandle privateNode_;
  ros::Publisher detect_pub_;
  ros::Publisher bathy_pub_;
  std::deque<std::string> cmd_resp_queue_;
};

#endif // NORBIT_CONNECTION_H
