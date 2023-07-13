#include "norbit_connection.h"

bool NorbitConnection::shutdown_ = false;

NorbitConnection::NorbitConnection() : privateNode_("~"), loop_rate(200.0) {
  updateParams();

  ROS_INFO("Connecting to norbit MBES at ip  : %s", params_.ip.c_str());
  ROS_INFO("  listening for bathy on port    : %i", params_.bathy_port);

  setupPubSub();

  waitForConnections();

  listenForCmd();

  initializeSonarParams();
  if(params_.pubWC()){
    receiveWC();
  }

  receiveBathy();
}

NorbitConnection::~NorbitConnection() {}

void NorbitConnection::updateParams() {
  privateNode_.param<std::string>("sensor_frame", params_.sensor_frame,
                                  "norbit");
  privateNode_.param<std::string>("ip", params_.ip, "10.1.70.133");
  privateNode_.param<int>("bathy_port", params_.bathy_port, 2210);
  privateNode_.param<int>("water_column_port", params_.water_column_port, 2211);
  privateNode_.param<int>("cmd_port", params_.cmd_port, 2209);

  // detections stuff
  privateNode_.param<std::string>("pointcloud_topic", params_.pointcloud_topic, "cloud");
  privateNode_.param<std::string>("bathymetric_topic",params_.bathymetric_topic, "bathymetric");
  privateNode_.param<std::string>("detections_topic", params_.detections_topic, "detections");
  privateNode_.param<std::string>("ranges_topic", params_.ranges_topic, "ranges");

  // Watercolumn stuff
  privateNode_.param<std::string>("norbit_watercolumn_topic",
                                  params_.norbit_watercolumn_topic, "water_column");

  privateNode_.param<std::string>("watercolumn_topic",
                                  params_.watercolumn_topic, "water_column/mb_wc");

  privateNode_.getParam("cmd_timeout", params_.cmd_timeout);
  privateNode_.getParam("disconnect_timeout", params_.disconnect_timeout);
  privateNode_.getParam("startup_settings", params_.startup_settings);
  privateNode_.getParam("shutdown_settings", params_.shutdown_settings);
}

void NorbitConnection::setupPubSub() {

  // publishers
  if (params_.pubPointcloud()){
    cloud_pub_ = node_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
        params_.pointcloud_topic, 1);
  }

  if (params_.pubDetections()){
    detect_pub_ = node_.advertise<acoustic_msgs::SonarDetections>(
          params_.detections_topic,1);
  }

  if (params_.pubRanges()) {
    ranges_pub_ = node_.advertise<acoustic_msgs::SonarRanges>(params_.ranges_topic, 1);
  }

  if (params_.pubBathymetric()){
    bathy_pub_ = node_.advertise<norbit_msgs::BathymetricStamped>(
      params_.bathymetric_topic, 1);
  }

  if(params_.pubNorbitWC()){
    norbit_wc_pub_ = node_.advertise<norbit_msgs::WaterColumnStamped>(
        params_.norbit_watercolumn_topic, 1);
  }

  if(params_.pubMultibeamWC()){
    wc_pub_ =  node_.advertise<acoustic_msgs::RawSonarImage>(
          params_.watercolumn_topic, 1);
  }

  // SRVs
  srv_map_["norbit_cmd"] = privateNode_.advertiseService(
      "norbit_cmd", &NorbitConnection::norbitCmdCallback, this);

  srv_map_["set_power"] = privateNode_.advertiseService(
      "set_power", &NorbitConnection::setPowerCallback, this);

  // timers
  disconnect_timer_ = privateNode_.createTimer(
      ros::Duration(params_.disconnect_timeout),
      &NorbitConnection::disconnectTimerCallback, this);
}

void NorbitConnection::waitForConnections(){
  while(!shutdown_ && !openConnections());

  ROS_INFO_COND(!shutdown_ , "Connection Estabilished");
}


bool NorbitConnection::openConnections() {
  try {
    sockets_.bathymetric = std::unique_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_service_));

    sockets_.bathymetric->connect(boost::asio::ip::tcp::endpoint(
        boost::asio::ip::address::from_string(params_.ip), params_.bathy_port));

    if(params_.pubWC()){
    sockets_.water_column = std::unique_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_service_));

    sockets_.water_column->connect(boost::asio::ip::tcp::endpoint(
        boost::asio::ip::address::from_string(params_.ip), params_.water_column_port));
    }
    sockets_.cmd = std::unique_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_service_));

    sockets_.cmd->connect(boost::asio::ip::tcp::endpoint(
        boost::asio::ip::address::from_string(params_.ip), params_.cmd_port));
    return true;
  }
  catch (const boost::exception& ex) {
      std::string info = boost::diagnostic_information(ex);
      ROS_WARN_THROTTLE(10.0,"Unable to connect to sonar.  Will continue trying.");
      //ROS_ERROR("unable to connect to sonar: %s",info.c_str());
      return false;
  }
}

void NorbitConnection::closeConnections() {
  sockets_.bathymetric->close();
  sockets_.bathymetric.reset();
  if(params_.pubWC()){
    sockets_.water_column->close();
    sockets_.water_column.reset();
  }
  sockets_.cmd->close();
  sockets_.cmd.reset();
}

void removeSubstrs(std::string &s, const std::string p) {
  std::string::size_type n = p.length();
  for (std::string::size_type i = s.find(p); i != std::string::npos;
       i = s.find(p))
    s.erase(i, n);
}

void NorbitConnection::initializeSonarParams() {
  for (auto param : params_.startup_settings) {
    while(!sendCmd(param.first, param.second).ack) {
      ROS_INFO_STREAM("Did not get ack for " << param.first << ". Resending");
    }
  }
}

norbit_msgs::CmdResp NorbitConnection::sendCmd(const std::string &cmd,
                                               const std::string &val) {

  norbit_msgs::CmdResp out;

  std::string message = cmd + " " + val;
  std::string key = cmd;

  // some of the norbit responses don't echo back set_<cmd> so we need to strip
  // it
  removeSubstrs(key, "set_");
  removeSubstrs(key, " ");

  ROS_INFO("command message sent: %s", message.c_str());
  sockets_.cmd->send(boost::asio::buffer(message));

  auto start_time = ros::WallTime::now();
  bool running = true;
  out.success = false;
  // Whether the ack matched what was expected from the command.
  // It seems like most of the callers use the ack field for testing success,
  // rather than the success field.
  out.ack = false;
  // holds most recent response; may not match the command. Used to monitor
  // whether any connection has been made.
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
        ROS_INFO_STREAM("..did not match key: " << key << ". Discarding response.");
        cmd_resp_queue_.pop_front();
      }
    } else {
      auto dt = ros::WallTime::now().toSec() - start_time.toSec();
      if (dt > params_.cmd_timeout) {
        ROS_ERROR("[%s] TIMEOUT -- no ACK received",
                  ros::this_node::getName().c_str());
        out.ack = false;
        running = false;
      }
    }
  } while (running);

  return out;
}

void NorbitConnection::listenForCmd() {

  // Most commands are echoed with 0x0d 0x0a (\r\n), but
  // set_ntp_server is termianted with 0x0a only.
  boost::asio::async_read_until(*sockets_.cmd, cmd_resp_buffer_, "\n",
                                boost::bind(&NorbitConnection::receiveCmd, this,
                                            boost::asio::placeholders::error));
}

void NorbitConnection::receiveCmd(const boost::system::error_code &err) {
  disconnect_timer_.setPeriod(ros::Duration(params_.disconnect_timeout), true);
  std::string line;
  std::istream is(&cmd_resp_buffer_);
  std::getline(is, line);
  cmd_resp_queue_.push_back(line);
  listenForCmd();
  return;
}

void NorbitConnection::receiveWC() {
  hdr_buff_.water_column.assign(0);

  sockets_.water_column->async_receive(
      boost::asio::buffer(hdr_buff_.water_column), 0,
      boost::bind(&NorbitConnection::wcHandler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void NorbitConnection::wcHandler(const boost::system::error_code &error,
                                  std::size_t bytes_transferred) {
  disconnect_timer_.setPeriod(ros::Duration(params_.disconnect_timeout), true);
  processHdrMsg(*sockets_.water_column,hdr_buff_.water_column);
  receiveWC();
  return;
}

void NorbitConnection::receiveBathy() {
  hdr_buff_.bathymetric.assign(0);
  sockets_.bathymetric->async_receive(
      boost::asio::buffer(hdr_buff_.bathymetric), 0,
      boost::bind(&NorbitConnection::bathyHandler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void NorbitConnection::bathyHandler(const boost::system::error_code &error,
                                  std::size_t bytes_transferred) {
  disconnect_timer_.setPeriod(ros::Duration(params_.disconnect_timeout), true);
  ROS_INFO("Got bathy!");
  processHdrMsg(*sockets_.bathymetric,hdr_buff_.bathymetric);
  receiveBathy();
  return;
}

void NorbitConnection::processHdrMsg(boost::asio::ip::tcp::socket & sock, boost::array<char, sizeof(norbit_msgs::CommonHeader)> &hdr){
  try {
    norbit_types::Message msg;
    if (msg.fromBoostArray(hdr)) {
      const unsigned int dataSize = msg.commonHeader().size - sizeof(norbit_msgs::CommonHeader);
      std::shared_ptr<char> dataPtr;
      dataPtr.reset(new char[dataSize]);
      size_t bytesRead =read(sock,boost::asio::buffer(dataPtr.get(), dataSize));

      if(msg.setBits(dataPtr)){
        if (msg.commonHeader().type == norbit_types::bathymetric) {
          bathyCallback(msg.getBathy());
        }
        if (msg.commonHeader().type == norbit_types::watercolum){
           wcCallback(msg.getWC());
        }
      }
      else ROS_WARN("Watercolumn Message failed CRC check:  Ignoring");

    }else{
      if(msg.commonHeader().version!=NORBIT_CURRENT_VERSION)
        ROS_WARN("Invalid version detected expected %i, got %i",
                 NORBIT_CURRENT_VERSION, msg.commonHeader().version);
      if(msg.commonHeader().preable==norbit_msgs::CommonHeader::NORBIT_PREAMBLE_KEY)
        ROS_WARN("Invalid header preable detected");
    }

  } catch (...) {
    ROS_ERROR("An unhandled exception occured in NorbitConnection::recHandler()");
  }
}



void NorbitConnection::bathyCallback(norbit_types::BathymetricData data) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr detections(
      new pcl::PointCloud<pcl::PointXYZI>);
  detections->header.frame_id = params_.sensor_frame;
  ros::Time stamp(data.bathymetricHeader().time);

  if (params_.pubPointcloud()){
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
        if ( data.data(i).quality_flag == 3) {
          detections->push_back(p);
        }
      }
    }
    pcl_conversions::toPCL(stamp, detections->header.stamp);
    cloud_pub_.publish(detections);
  }

  auto bathy_msg = data.getRosMsg(params_.sensor_frame);
  if (params_.pubBathymetric()) {
    bathy_pub_.publish(bathy_msg);
  }

  if (params_.pubDetections()) {
    acoustic_msgs::SonarDetections detections_msg;
    norbit::conversions::bathymetric2SonarDetections(bathy_msg, detections_msg);
    detect_pub_.publish(detections_msg);
  }

  if (params_.pubRanges()) {
    acoustic_msgs::SonarRanges ranges_msg;
    norbit::conversions::bathymetric2SonarRanges(bathy_msg, ranges_msg);
    ranges_pub_.publish(ranges_msg);
  }

  return;
}

void NorbitConnection::wcCallback(norbit_types::WaterColumnData data){
  auto norb_wc_msg = data.getRosMsg(params_.sensor_frame);
  if(params_.pubNorbitWC())
    norbit_wc_pub_.publish(norb_wc_msg);

  if(params_.pubMultibeamWC()){
    acoustic_msgs::RawSonarImage::Ptr hydro_wc_msg(new acoustic_msgs::RawSonarImage);
    norbit::conversions::norbitWC2RawSonarImage(norb_wc_msg, *hydro_wc_msg);
    wc_pub_.publish(hydro_wc_msg);
  }
}

void NorbitConnection::disconnectTimerCallback(const ros::TimerEvent& event){
  ROS_INFO("No Messages received for a while. Checking Connections");
  // When the sonar powers itself off in air, there won't be any data but
  // the connection may still be fine. So, issue a query to check.
  // NOTE: It is important to NOT check that ack was successful here -- we
  //   just need aliveness. The timer is called in a separate thread from the initial
  //   parameter initialization, and if those are slow enough to time this out,
  //   (and timeouts were chosen poorly) it has gotten stuck in an infinite loop
  //   with the main thread discarding responses to the set_power command,
  //   and the timer thread resetting the connection ...
  if(sendCmd("set_power", "").resp == "") {
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
  while (!shutdown_) {
    spin_once();
  }
  ROS_WARN("[%s] shutting down:  executing shutdown parameters",ros::this_node::getName().c_str());
  for (auto param : params_.shutdown_settings) {
    sendCmd(param.first, param.second);
  }
  ros::shutdown();
}
