#include <norbit/ros_wrapper.hpp>

NorbitRos::NorbitRos()
    : Node("norbit_ros_node")
{

    //! Load param
    updateParams();

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
    this->declare_parameter<std::string>("norbit_watercolumn_topic", "water_column");
    this->get_parameter("norbit_watercolumn_topic", params_.norbit_watercolumn_topic);

    this->declare_parameter<std::string>("watercolumn_topic", "water_column/mb_wc");
    this->get_parameter("watercolumn_topic", params_.watercolumn_topic);
    
    this->declare_parameter<double>("cmd_timeout", 0.5);
    this->get_parameter("cmd_timeout", params_.cmd_timeout);

    std::string str_param;
    this->declare_parameter<std::string>("startup_settings.set_power", "");
    this->get_parameter("startup_settings.set_power", str_param);
    params_.startup_settings["set_power"] = str_param;

    this->declare_parameter<std::string>("startup_settings.set_gate_mode", "");
    this->get_parameter("startup_settings.set_gate_mode", str_param);
    params_.startup_settings["set_gate_mode"] = str_param;

    this->declare_parameter<std::string>("startup_settings.set_range", "");
    this->get_parameter("startup_settings.set_range", str_param);
    params_.startup_settings["set_range"] = str_param;

    this->declare_parameter<std::string>("startup_settings.set_time_source", "");
    this->get_parameter("startup_settings.set_time_source", str_param);
    params_.startup_settings["set_time_source"] = str_param;

    //! TODO: check if the value empty before send to sonar?
    this->declare_parameter<std::string>("startup_settings.set_ntp_server", "");
    this->get_parameter("startup_settings.set_ntp_server", str_param);
    params_.startup_settings["set_ntp_server"] = str_param;

    this->declare_parameter<std::string>("shutdown_settings.set_power", "");
    this->get_parameter("shutdown_settings.set_power", str_param);
    params_.shutdown_settings["set_power"] = str_param;


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

void NorbitRos::setupSubpub()
{
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 20);
}