#ifndef NORBIT_CONNECTION_H
#define NORBIT_CONNECTION_H

#include <iostream>
#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "norbit_types/header.h"
#include "norbit_types/bathymetric_data.h"
#include "norbit_types/message.h"


using namespace boost::asio;
using ip::tcp;

struct ConnectionParams{
  std::string ip;
  int bathy_port;
  std::string sensor_frame;
  std::string pointcloud_topic;
  std::string bathymetric_topic;
};

class NorbitConnection{
public:
  NorbitConnection();
  void updateParams();
  void receive();
  void recHandler(
      const boost::system::error_code& error, // Result of operation.
      std::size_t bytes_transferred           // Number of bytes received.
    );
  void spin();
  void bathyCallback(norbit_types::BathymetricData data);
protected:
  std::unique_ptr<tcp::socket> socket_;
  boost::asio::io_service io_service_;
  boost::array<char, sizeof(norbit_types::Header)> recv_buffer_;
  boost::array<char, 50000> dataBuffer_;
  //std::string sensor_frame_;
  ConnectionParams params_;
  ros::NodeHandle node_;
  ros::NodeHandle privateNode_;
  ros::Publisher detect_pub_;
  ros::Publisher bathy_pub_;
};

#endif // NORBIT_CONNECTION_H
