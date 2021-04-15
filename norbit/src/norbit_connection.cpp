#include "norbit_connection.h"

NorbitConnection::NorbitConnection() : privateNode_("~") {
  updateParams();

  ROS_INFO("Connecting to norbit MBES at ip  : %s", params_.ip.c_str());
  ROS_INFO("  listening for bathy on port    : %i", params_.bathy_port);

  detect_pub_ = node_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      params_.pointcloud_topic, 1);

  bathy_pub_ = node_.advertise<norbit_msgs::BathymetricStamped>(
      params_.bathymetric_topic, 1);

  setupConnections();
  listenForCmd();

  for (auto param : params_.startup_settings) {
    sendCmd(param.first, param.second);
  }

  receive();
}

NorbitConnection::~NorbitConnection() {}
void NorbitConnection::updateParams() {
  privateNode_.param<std::string>("sensor_frame", params_.sensor_frame,
                                  "norbit");
  privateNode_.param<std::string>("ip", params_.ip, "192.168.53.24");
  privateNode_.param<int>("bathy_port", params_.bathy_port, 2210);
  privateNode_.param<int>("cmd_port", params_.cmd_port, 2209);
  privateNode_.param<std::string>("pointcloud_topic", params_.pointcloud_topic,
                                  "detections");
  privateNode_.param<std::string>("bathymetric_topic",
                                  params_.bathymetric_topic, "bathymetric");
  privateNode_.getParam("cmd_timeout", params_.cmd_timeout);
  privateNode_.getParam("startup_settings", params_.startup_settings);
  privateNode_.getParam("shutdown_settings", params_.shutdown_settings);
}

void NorbitConnection::setupConnections() {
  sockets_.bathymetric = std::unique_ptr<boost::asio::ip::tcp::socket>(
      new boost::asio::ip::tcp::socket(io_service_));
  sockets_.bathymetric->connect(boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string(params_.ip), params_.bathy_port));

  sockets_.cmd = std::unique_ptr<boost::asio::ip::tcp::socket>(
      new boost::asio::ip::tcp::socket(io_service_));
  sockets_.cmd->connect(boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string(params_.ip), params_.cmd_port));
}

void removeSubstrs(std::string &s, const std::string p) {
  std::string::size_type n = p.length();
  for (std::string::size_type i = s.find(p); i != std::string::npos;
       i = s.find(p))
    s.erase(i, n);
}

void NorbitConnection::sendCmd(const std::string &cmd, const std::string &val) {

  std::string message = cmd + " " + val;
  std::string key = cmd;
  removeSubstrs(key, "set_");
  removeSubstrs(key, " ");

  ROS_INFO("command message sent: %s", message.c_str());
  sockets_.cmd->send(boost::asio::buffer(message));

  auto start_time = ros::WallTime::now();
  ros::Duration timeout(.5);
  bool running = true;
  std::string last = "";
  do {
    io_service_.run_one();
    if (cmd_resp_queue_.size() > 0) {
      last = cmd_resp_queue_.front();
      if (cmd_resp_queue_.front().find(key) != std::string::npos) {
        ROS_INFO("ACK Received: %s", cmd_resp_queue_.front().c_str());
        cmd_resp_queue_.pop_front();
        running = false;
      } else {
        cmd_resp_queue_.pop_front();
      }
    } else {
      if (ros::WallTime::now().toSec() - start_time.toSec() >
          params_.cmd_timeout) {

        if (last != "") {
          ROS_ERROR("[%s] received bad ACK: %s",
                    ros::this_node::getName().c_str(), last.c_str());
        } else {
          ROS_ERROR("[%s] TIMEOUT -- no ACK received",
                    ros::this_node::getName().c_str());
        }
        running = false;
      }
    }
  } while (running);
}

void NorbitConnection::listenForCmd() {

  boost::asio::async_read_until(*sockets_.cmd, cmd_resp_buffer_, "\r\n",
                                boost::bind(&NorbitConnection::receiveCmd, this,
                                            boost::asio::placeholders::error));
}

void NorbitConnection::receiveCmd(const boost::system::error_code &err) {
  std::string line;
  std::istream is(&cmd_resp_buffer_);
  std::getline(is, line);
  cmd_resp_queue_.push_back(line);
  listenForCmd();
  return;
}

void NorbitConnection::receive() {
  recv_buffer_.assign(0);
  sockets_.bathymetric->async_receive(
      boost::asio::buffer(recv_buffer_), 0,
      boost::bind(&NorbitConnection::recHandler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void NorbitConnection::recHandler(const boost::system::error_code &error,
                                  std::size_t bytes_transferred) {
  norbit_types::Message msg;
  if (msg.fromBoostArray(recv_buffer_)) {
    const unsigned int dataSize =
        msg.getHeader().size - sizeof(norbit_types::Header);
    size_t bytesRead =
        read(*sockets_.bathymetric, boost::asio::buffer(dataBuffer_, dataSize));
    std::shared_ptr<char> dataPtr;
    dataPtr.reset(new char[dataSize]);
    memcpy(dataPtr.get(), &dataBuffer_, bytesRead);
    msg.setBits(dataPtr);

    if (msg.getHeader().type == norbit_types::bathymetric) {
      bathyCallback(msg.getBathy());
    }
  }
  receive();
  return;
}

void NorbitConnection::bathyCallback(norbit_types::BathymetricData data) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr detections(
      new pcl::PointCloud<pcl::PointXYZI>);
  detections->header.frame_id = params_.sensor_frame;
  ros::Time stamp(data.getHeader().time);

  for (size_t i = 0; i < data.getHeader().N; i++) {
    if (data.getData(i).sample_number > 1) {
      float range = float(data.getData(i).sample_number) *
                    data.getHeader().snd_velocity /
                    (2.0 * data.getHeader().sample_rate);
      pcl::PointXYZI p;
      p.x = range * sinf(data.getHeader().tx_angle);
      p.y = range * sinf(data.getData(i).angle);
      p.z = range * cosf(data.getData(i).angle);
      p.intensity = float(data.getData(i).intensity) / 1e9f;
      if (data.getData(i).quality_flag == 3) {
        detections->push_back(p);
      }
    }
  }
  pcl_conversions::toPCL(stamp, detections->header.stamp);
  detect_pub_.publish(detections);
  bathy_pub_.publish(data.getRosMsg(params_.sensor_frame));
  return;
}

void NorbitConnection::spin() {
  // io_service_.run();
  while (ros::ok()) {
    io_service_.run_one();
    ros::spinOnce();
  }

  ROS_INFO("shutting down...");
  for (auto param : params_.shutdown_settings) {
    sendCmd(param.first, param.second);
  }
}
