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

#include "../include/norbit/norbit_types/header.h"
#include "../include/norbit/norbit_types/bathymetric_data.h"
#include "../include/norbit/norbit_types/message.h"


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

NorbitConnection::NorbitConnection():
privateNode_("~")
{
  updateParams();
  detect_pub_ = node_.advertise<pcl::PointCloud<pcl::PointXYZI>> (params_.pointcloud_topic, 1);
  bathy_pub_  = node_.advertise<norbit_msgs::BathymetricStamped> (params_.bathymetric_topic,1);
  socket_ = std::unique_ptr<tcp::socket>(new tcp::socket(io_service_) );
  ROS_INFO("Connecting to norbit MBES at ip  : %s",params_.ip.c_str());
  ROS_INFO("  listening for bathy on port    : %i",params_.bathy_port);
  socket_->connect(tcp::endpoint( boost::asio::ip::address::from_string(params_.ip), params_.bathy_port ));

  receive();
}

void NorbitConnection::updateParams(){
  privateNode_.param<std::string>("sensor_frame",params_.sensor_frame,"/norbit/multibeam");
  privateNode_.param<std::string>("ip",params_.ip,"192.168.53.24");
  privateNode_.param<int>("bathy_port",params_.bathy_port,2210);
  privateNode_.param<std::string>("pointcloud_topic",params_.pointcloud_topic,"detections");
  privateNode_.param<std::string>("bathymetric_topic",params_.bathymetric_topic,"bathymetric");
}

void NorbitConnection::receive(){
  recv_buffer_.assign(0);
  socket_->async_receive(boost::asio::buffer(recv_buffer_), 0,
                         boost::bind(&NorbitConnection::recHandler, this, boost::asio::placeholders::error,
                                     boost::asio::placeholders::bytes_transferred));
}

void NorbitConnection::recHandler(const boost::system::error_code &error, std::size_t bytes_transferred){
  norbit_types::Message msg;
  if(msg.fromBoostArray(recv_buffer_)){
    const unsigned int dataSize = msg.getHeader().size-sizeof(norbit_types::Header);
    size_t bytesRead = read(*socket_,boost::asio::buffer(dataBuffer_,dataSize));
    std::shared_ptr<char> dataPtr;
    dataPtr.reset(new char[dataSize]);
    memcpy(dataPtr.get(),&dataBuffer_,bytesRead);
    msg.setBits(dataPtr);

    if(msg.getHeader().type==norbit_types::bathymetric){
      bathyCallback(msg.getBathy());
    }
  }
  receive();
  return;
}

void NorbitConnection::bathyCallback(norbit_types::BathymetricData data){
  pcl::PointCloud<pcl::PointXYZI>::Ptr detections (new pcl::PointCloud<pcl::PointXYZI>);
  detections->header.frame_id = params_.sensor_frame;
  ros::Time stamp(data.getHeader().time);

  for(size_t i = 0; i<data.getHeader().N; i++){
    if(data.getData(i).sample_number>1){
      float range = float(data.getData(i).sample_number) * data.getHeader().snd_velocity/(2.0*data.getHeader().sample_rate);
      pcl::PointXYZI p;
      p.x=0;
      p.y=range * sinf(data.getData(i).angle);;
      p.z=range * cosf(data.getData(i).angle);
      p.intensity=float(data.getData(i).intensity)/1e9f;
      detections->push_back(p);
    }
  }
  pcl_conversions::toPCL(stamp, detections->header.stamp);
  detect_pub_.publish(detections);
  bathy_pub_.publish(data.getRosMsg(params_.sensor_frame));
  return;
}

void NorbitConnection::spin(){
  //io_service_.run();
  while(ros::ok())
    io_service_.run_one();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "norbit_node");
  NorbitConnection con;
  con.spin();

}

