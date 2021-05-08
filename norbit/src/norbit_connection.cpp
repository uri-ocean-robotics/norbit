#include "norbit_connection.h"

NorbitConnection::NorbitConnection() : privateNode_("~"), loop_rate(200.0) {
  updateParams();

  ROS_INFO("Connecting to norbit MBES at ip  : %s", params_.ip.c_str());
  ROS_INFO("  listening for bathy on port    : %i", params_.bathy_port);

  setupPubSub();

  waitForConnections();

  listenForCmd();

  initializeSonarParams();

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

void NorbitConnection::setupPubSub() {

  // publishers
  detect_pub_ = node_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      params_.pointcloud_topic, 1);

  bathy_pub_ = node_.advertise<norbit_msgs::BathymetricStamped>(
      params_.bathymetric_topic, 1);

  // SRVs
  srv_map_["norbit_cmd"] = privateNode_.advertiseService(
      "norbit_cmd", &NorbitConnection::norbitCmdCallback, this);

  srv_map_["set_power"] = privateNode_.advertiseService(
      "set_power", &NorbitConnection::setPowerCallback, this);

  // timers
  disconnect_timer_ = privateNode_.createTimer(ros::Duration(1.0),
                           &NorbitConnection::disconnectTimerCallback,
                           this);
}

void NorbitConnection::waitForConnections(){
  while(!openConnections());
}


bool NorbitConnection::openConnections() {
  try {
    sockets_.bathymetric = std::unique_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_service_));
    sockets_.bathymetric->connect(boost::asio::ip::tcp::endpoint(
        boost::asio::ip::address::from_string(params_.ip), params_.bathy_port));

    sockets_.cmd = std::unique_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_service_));

    sockets_.cmd->connect(boost::asio::ip::tcp::endpoint(
        boost::asio::ip::address::from_string(params_.ip), params_.cmd_port));
    return true;
  }
  catch (const boost::exception& ex) {
      std::string info = boost::diagnostic_information(ex);
      ROS_ERROR("unable to connect to sonar: %s",info.c_str());
      return false;
  }
}

void NorbitConnection::closeConnections() {
  sockets_.bathymetric->close();
  sockets_.bathymetric.reset();
  sockets_.cmd->close();
  sockets_.cmd.reset();
}

void removeSubstrs(std::string &s, const std::string p) {
  std::string::size_type n = p.length();
  for (std::string::size_type i = s.find(p); i != std::string::npos;
       i = s.find(p))
    s.erase(i, n);
}

void NorbitConnection::initializeSonarParams(){
  for (auto param : params_.startup_settings) {
    while(!sendCmd(param.first, param.second).ack){
      ros::Duration timeout(params_.cmd_timeout);
      timeout.sleep();
    }
  }
}

norbit_msgs::CmdResp NorbitConnection::sendCmd(const std::string &cmd,
                                               const std::string &val) {

  norbit_msgs::CmdResp out;

  std::string message = cmd + " " + val;
  std::string key = cmd;

  // some of the norbit reponses don't echo back set_<cmd> so we need to strip
  // it
  removeSubstrs(key, "set_");
  removeSubstrs(key, " ");

  ROS_INFO("command message sent: %s", message.c_str());
  sockets_.cmd->send(boost::asio::buffer(message));

  auto start_time = ros::WallTime::now();
  bool running = true;
  out.success = false;
  out.ack = false;
  out.resp = "";
  do {
    spin_once();
    if (cmd_resp_queue_.size() > 0) {
      out.resp = cmd_resp_queue_.front();
      if (cmd_resp_queue_.front().find(key) != std::string::npos) {
        ROS_INFO("ACK Received: %s", cmd_resp_queue_.front().c_str());
        cmd_resp_queue_.pop_front();
        out.success = true;
        out.ack = true;
        running = false;
      } else {
        cmd_resp_queue_.pop_front();
      }
    } else {
      if (ros::WallTime::now().toSec() - start_time.toSec() >
          params_.cmd_timeout) {

        if (out.resp != "") {
          ROS_ERROR("[%s] received bad ACK: %s",
                    ros::this_node::getName().c_str(), out.resp.c_str());
          out.ack = true;
        } else {
          ROS_ERROR("[%s] TIMEOUT -- no ACK received",
                    ros::this_node::getName().c_str());
          out.ack = false;
        }
        running = false;
      }
    }
  } while (running);

  return out;
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
  disconnect_timer_.setPeriod(ros::Duration(1.0),true);
  try {
    norbit_types::Message msg;
    if (msg.fromBoostArray(recv_buffer_)) {
      const unsigned int dataSize =
          msg.commonHeader().size - sizeof(norbit_msgs::CommonHeader);
      size_t bytesRead =read(*sockets_.bathymetric,
                             boost::asio::buffer(dataBuffer_, dataSize));
      std::shared_ptr<char> dataPtr;
      dataPtr.reset(new char[dataSize]);
      memcpy(dataPtr.get(), &dataBuffer_, bytesRead);

      if(msg.setBits(dataPtr)){
        if (msg.commonHeader().type == norbit_types::bathymetric) {
          bathyCallback(msg.getBathy());
        }
      }
      else ROS_WARN("Message failed CRC check:  Ignoring");
    }else{
      if(msg.commonHeader().version==NORBIT_CURRENT_VERSION)
        ROS_WARN("Invalid version detected expected %i, got %i",
                 NORBIT_CURRENT_VERSION, msg.commonHeader().version);
      if(msg.commonHeader().preable==norbit_msgs::CommonHeader::NORBIT_PREAMBLE_KEY)
        ROS_WARN("Invalid header preable detected");
    }

  } catch (...) {
    ROS_ERROR("An unhandled exception occured in NorbitConnection::recHandler()");
  }

  receive();
  return;
}

void NorbitConnection::bathyCallback(norbit_types::BathymetricData data) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr detections(
      new pcl::PointCloud<pcl::PointXYZI>);
  detections->header.frame_id = params_.sensor_frame;
  ros::Time stamp(data.bathymetricHeader().time);

  for (size_t i = 0; i < data.bathymetricHeader().N; i++) {
    if (data.data(i).sample_number > 1) {
      float range = float(data.data(i).sample_number) *
                    data.bathymetricHeader().snd_velocity /
                    (2.0 * data.bathymetricHeader().sample_rate);
      pcl::PointXYZI p;
      p.x = range * sinf(data.bathymetricHeader().tx_angle);
      p.y = range * sinf(data.data(i).angle);
      p.z = range * cosf(data.data(i).angle);
      p.intensity = float(data.data(i).intensity) / 1e9f;
      if (data.data(i).quality_flag == 3) {
        detections->push_back(p);
      }
    }
  }
  pcl_conversions::toPCL(stamp, detections->header.stamp);
  detect_pub_.publish(detections);
  bathy_pub_.publish(data.getRosMsg(params_.sensor_frame));
  return;
}

void NorbitConnection::disconnectTimerCallback(const ros::TimerEvent& event){
  ROS_INFO("No Messages received for a while.   Checking Connections");
  if(!sendCmd("set_power", "").ack){
    ROS_ERROR("Sonar disconnected: restarting connections");
    closeConnections();
    waitForConnections();
    initializeSonarParams();
  }
  return;
}

bool NorbitConnection::norbitCmdCallback(
    norbit_msgs::NorbitCmd::Request &req,
    norbit_msgs::NorbitCmd::Response &resp) {
  resp.resp = sendCmd(req.cmd, req.val);
  return resp.resp.ack;
}

bool NorbitConnection::setPowerCallback(norbit_msgs::SetPower::Request &req,
                                        norbit_msgs::SetPower::Response &resp) {
  resp.resp = sendCmd("set_power", std::to_string(req.on));
  return resp.resp.success;
}

void NorbitConnection::spin_once() {
  io_service_.poll_one();
  ros::spinOnce();
  loop_rate.sleep();
}

void NorbitConnection::spin() {

  while (ros::ok()) {
    spin_once();
  }

  ROS_INFO("shutting down...");
  for (auto param : params_.shutdown_settings) {
    sendCmd(param.first, param.second);
  }
}
