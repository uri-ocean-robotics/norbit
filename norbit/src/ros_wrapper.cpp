#include <norbit/ros_wrapper.hpp>

using namespace std::chrono_literals;

NorbitRos::NorbitRos()
    : Node("norbit_ros2_node")
{

    //! Load param
    updateParams();

    //! Setup ROS
    setupROS();

    //! Setup TCP 
    setupTCP();
}

NorbitRos::~NorbitRos()
{
    // std::cout<<"~NorbitRos  called now\n";
    closeSonar();
}

void NorbitRos::updateParams()
{
    //! SONAR param
    this->declare_parameter<std::string>("sensor_frame", DEFAULT_SENSOR_FRAME);
    this->get_parameter("sensor_frame", params_.sensor_frame);

    this->declare_parameter<std::string>("ip", DEFAULT_IP);
    this->get_parameter("ip", params_.ip);

    this->declare_parameter<int>("bathy_port", DEFAULT_BATHY_PORT);
    this->get_parameter("bathy_port", params_.bathy_port);

    this->declare_parameter<int>("water_column_port", DEFAULT_WATER_COLUMN_PORT);
    this->get_parameter("water_column_port", params_.water_column_port);

    this->declare_parameter<int>("cmd_port", DEFAULT_CMD_PORT);
    this->get_parameter("cmd_port", params_.cmd_port);

    //! detections stuff 
    //! TODO: no need to load param for topics, we can do remapping
    this->declare_parameter<std::string>("pointcloud_topic", "cloud");
    this->get_parameter("pointcloud_topic", params_.pointcloud_topic);

    this->declare_parameter<std::string>("bathymetric_topic", "bathymetric");
    this->get_parameter("bathymetric_topic", params_.bathymetric_topic);

    this->declare_parameter<std::string>("detections_topic", "detections");
    this->get_parameter("detections_topic", params_.detections_topic);

    this->declare_parameter<std::string>("ranges_topic", "ranges");
    this->get_parameter("ranges_topic", params_.ranges_topic);

    //! Watercolumn stuff
    this->declare_parameter<std::string>("norbit_watercolumn_topic", "");
    this->get_parameter("norbit_watercolumn_topic", params_.norbit_watercolumn_topic);

    this->declare_parameter<std::string>("watercolumn_topic", "");
    this->get_parameter("watercolumn_topic", params_.watercolumn_topic);
    
    this->declare_parameter<double>("cmd_timeout", 0.5);
    this->get_parameter("cmd_timeout", params_.cmd_timeout);

    this->declare_parameter<double>("disconnect_timeout", 1.0);
    this->get_parameter("disconnect_timeout", params_.disconnect_timeout);    

    //! TODO: auto loading the settings
    std::string str_param;
    this->declare_parameter<std::string>("startup_settings.set_power", "");
    this->get_parameter("startup_settings.set_power", str_param);
    if(!str_param.empty()) {
        params_.startup_settings["set_power"] = str_param;
    }

    this->declare_parameter<std::string>("startup_settings.set_gate_mode", "");
    this->get_parameter("startup_settings.set_gate_mode", str_param);
    if(!str_param.empty()) {
        params_.startup_settings["set_gate_mode"] = str_param;
    }

    this->declare_parameter<std::string>("startup_settings.set_range", "");
    this->get_parameter("startup_settings.set_range", str_param);
    if(!str_param.empty()) {
        params_.startup_settings["set_range"] = str_param;
    }    

    this->declare_parameter<std::string>("startup_settings.set_time_source", "");
    this->get_parameter("startup_settings.set_time_source", str_param);
    if(!str_param.empty()) {
        params_.startup_settings["set_time_source"] = str_param;
    }

    this->declare_parameter<std::string>("startup_settings.set_ntp_server", "");
    this->get_parameter("startup_settings.set_ntp_server", str_param);
    if(!str_param.empty()) {
        params_.startup_settings["set_ntp_server"] = str_param;
    }

    this->declare_parameter<std::string>("startup_settings.set_rate", "");
    this->get_parameter("startup_settings.set_rate", str_param);
    if(!str_param.empty()) {
        params_.startup_settings["set_rate"] = str_param;
    }

    this->declare_parameter<std::string>("shutdown_settings.set_power", "");
    this->get_parameter("shutdown_settings.set_power", str_param);
    if(!str_param.empty()) {
        params_.shutdown_settings["set_power"] = str_param;
    }


    //! DEBUG:
    std::cout<<"SONAR config: \n";
    std::cout<<"   sensor_frame: "      << params_.sensor_frame      <<"\n";
    std::cout<<"   ip: "                << params_.ip                <<"\n";
    std::cout<<"   bathy_port: "        << params_.bathy_port        <<"\n";
    std::cout<<"   water_column_port: " << params_.water_column_port <<"\n";
    std::cout<<"   cmd_port: "          << params_.cmd_port          <<"\n";
    std::cout<<"\n";      

    std::cout<<"Detections stuff: \n";
    std::cout<<"   pointcloud_topic: "   << params_.pointcloud_topic  <<"\n";
    std::cout<<"   bathymetric_topic: "  << params_.bathymetric_topic <<"\n";
    std::cout<<"   detections_topic: "   << params_.detections_topic  <<"\n";
    std::cout<<"   ranges_topic: "       << params_.ranges_topic      <<"\n";
    std::cout<<"\n";   

    std::cout<<"Watercolumn stuff: \n";
    std::cout<<"   norbit_watercolumn_topic: "  << params_.norbit_watercolumn_topic <<"\n";
    std::cout<<"   watercolumn_topic: "         << params_.watercolumn_topic        <<"\n";
    std::cout<<"   cmd_timeout: "               << params_.cmd_timeout              <<"\n";
    std::cout<<"   startup_settings:\n";
    for (const auto &[key, value] : params_.startup_settings) {
        std::cout<<"        " << key <<": " << value << "\n";
    }
    std::cout<<"   shutdown_settings:\n";
    for (const auto &[key, value] : params_.shutdown_settings) {
        std::cout<<"        " << key <<": " << value << "\n";
    }
    std::cout<<"\n";                    
}

void NorbitRos::setupROS()
{
    // ===================================================================== //
    // ros publishers
    // ===================================================================== //

    //! TODO: change to check if any subscription exists

    if (params_.pubPointcloud()){
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            params_.pointcloud_topic, 1);
    }

    if (params_.pubDetections()){
        detect_pub_ = this->create_publisher<marine_acoustic_msgs::msg::SonarDetections>(
            params_.detections_topic, 1);            
    }

    if (params_.pubRanges()){
        ranges_pub_ = this->create_publisher<marine_acoustic_msgs::msg::SonarRanges>(
            params_.ranges_topic, 1);            
    }

    if (params_.pubBathymetric()){
        bathy_pub_ = this->create_publisher<norbit_msgs::msg::BathymetricStamped>(
            params_.bathymetric_topic, 1);            
    }    

    if (params_.pubNorbitWC()){
        norbit_wc_pub_ = this->create_publisher<norbit_msgs::msg::WaterColumnStamped>(
            params_.norbit_watercolumn_topic, 1);            
    }  

    if (params_.pubMultibeamWC()){
        wc_pub_ = this->create_publisher<marine_acoustic_msgs::msg::RawSonarImage>(
            params_.watercolumn_topic, 1);            
    }  

    // ===================================================================== //
    // ros services
    // ===================================================================== //

    norbit_cmd_srv_ = this->create_service<norbit_msgs::srv::NorbitCmd>(
        "~/norbit_cmd", std::bind(
            &NorbitRos::norbitCmdCallback, this, 
            std::placeholders::_1, std::placeholders::_2));

    set_power_srv_ = this->create_service<norbit_msgs::srv::SetPower>(
        "~/set_power", std::bind(
            &NorbitRos::setPowerCallback, this, 
            std::placeholders::_1, std::placeholders::_2));            

    // ===================================================================== //
    // disconnect_timer (or watchdog timer), if no received in given timeout, 
    // it will re-connect
    // ===================================================================== //

    disconnect_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(params_.disconnect_timeout),
        std::bind(&NorbitRos::disconnectTimerCallback, this));

    // stop the timer since we want to create sonar connection first
    disconnect_timer_->cancel();
}

void NorbitRos::setupTCP()
{
    // all the connection will be established when create the tcp handler
    tcp_handler_ = std::make_shared<TCPSocketHandler>(params_);

    //! Set up callback for received data from the socket handler (actually parsed)
    tcp_handler_->setCallbackBathy(
        std::bind(&NorbitRos::callbackBathty, this, std::placeholders::_1)
    );

    tcp_handler_->setCallbackWC(
        std::bind(&NorbitRos::callbackWC, this, std::placeholders::_1)
    );

    // setup all the connections
    waitForConnections();
}

void NorbitRos::waitForConnections()
{
    // setup all the sockets
    tcp_handler_->setupConnection();

    // initialize the sonar param: power on the sonar, and more ....
    initializeSonarParams();

    // since sonar data is streaming now, we can start the timer
    rclcpp::sleep_for(std::chrono::seconds(3)); 
    disconnect_timer_->reset();
}

void NorbitRos::initializeSonarParams(){
  for (auto param : params_.startup_settings) {
    while(!tcp_handler_->sendCmd(param.first, param.second).ack){
        RCLCPP_INFO(this->get_logger(), "param:%s not get ACK", param.first.c_str());
    }
  }
}

void NorbitRos::closeSonar()
{
    // std::cout<<" closeSonar begin...\n";
    for (auto param : params_.shutdown_settings) {
        tcp_handler_->sendCmd(param.first, param.second);
    }
    // std::cout<<" closeSonar end...\n";
}

void NorbitRos::callbackBathty(norbit_types::BathymetricData data) 
{
    // since we got the data, reset the disconnect timer counting duration
    disconnect_timer_->reset();

    // RCLCPP_INFO(this->get_logger(), "parsed BathymetricData: time=%.9f", 
    //     data.bathymetricHeader().time);

    // ===================================================================== //
    // Publish the Pointcloud
    // ===================================================================== //

    if (params_.pubPointcloud()){

            
        // fill up the pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr detections(
            new pcl::PointCloud<pcl::PointXYZI>);
        for (size_t i = 0; i < data.bathymetricHeader().beam_number; i++) {
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

        // convert the pcl to ros msg
        auto stamp = doubleToRosStamp(data.bathymetricHeader().time);
        auto ros_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*detections, *ros_cloud);
        ros_cloud->header.stamp = stamp;
        ros_cloud->header.frame_id = params_.sensor_frame;
        cloud_pub_->publish(std::move(ros_cloud));

        auto sys_time = this->now().seconds();
        if(abs(data.bathymetricHeader().time-sys_time) > 10) {
            RCLCPP_WARN(this->get_logger(), "Time sync failed: norbit time=%.9f, sys time=%.9f", 
                data.bathymetricHeader().time, sys_time);
        }
    }

    // ===================================================================== //
    // Publish other derived msg
    // ===================================================================== //

    auto bathy_msg = data.getRosMsg(params_.sensor_frame);
    if (params_.pubBathymetric()) {
        bathy_pub_->publish(bathy_msg);
    }

    if (params_.pubDetections()) {
        marine_acoustic_msgs::msg::SonarDetections detections_msg;
        norbit::conversions::bathymetric2SonarDetections(bathy_msg, detections_msg);
        detect_pub_->publish(detections_msg);
    }

    if (params_.pubRanges()) {
        marine_acoustic_msgs::msg::SonarRanges ranges_msg;
        norbit::conversions::bathymetric2SonarRanges(bathy_msg, ranges_msg);
        ranges_pub_->publish(ranges_msg);
    }

}

void NorbitRos::callbackWC(norbit_types::WaterColumnData data) 
{
    // since we got the data, reset the disconnect timer counting duration
    disconnect_timer_->reset();

    auto norb_wc_msg = data.getRosMsg(params_.sensor_frame);
    if(params_.pubNorbitWC()) {
        norbit_wc_pub_->publish(norb_wc_msg);
    }

    if(params_.pubMultibeamWC()){
        auto hydro_wc_msg = std::make_shared<marine_acoustic_msgs::msg::RawSonarImage>();

        norbit::conversions::norbitWC2RawSonarImage(norb_wc_msg, *hydro_wc_msg);
        wc_pub_->publish(*hydro_wc_msg);
    }

    // RCLCPP_INFO(this->get_logger(), "parsed WaterColumnData: time=%.9f", 
    //     stampTodouble(norb_wc_msg.header.stamp));
}

bool NorbitRos::norbitCmdCallback(
    const std::shared_ptr<norbit_msgs::srv::NorbitCmd::Request> req,
    const std::shared_ptr<norbit_msgs::srv::NorbitCmd::Response> resp)
{
    resp->resp = tcp_handler_->sendCmd(req->cmd, req->val);
    return resp->resp.ack;
}          

bool NorbitRos::setPowerCallback(
    const std::shared_ptr<norbit_msgs::srv::SetPower::Request> req,
    const std::shared_ptr<norbit_msgs::srv::SetPower::Response> resp)
{
    resp->resp = tcp_handler_->sendCmd("set_power", std::to_string(req->on));
    return resp->resp.success;
}

void NorbitRos::disconnectTimerCallback()
{
    RCLCPP_WARN(this->get_logger(), "No Messages received for a while. Checking Connections");

    if(tcp_handler_->sendCmd("set_power", "").resp == "") {
        RCLCPP_ERROR(this->get_logger(), "Sonar disconnected: restarting connections");
        // stop the timer
        disconnect_timer_->cancel();
        // close the TCP socket
        tcp_handler_->closeConnection();
        // setup the TCP socket connection again
        waitForConnections();
    }
}