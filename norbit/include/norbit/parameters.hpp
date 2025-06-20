#pragma once

#include <string>
#include <map>

struct ConnectionParams {
  std::string ip;
  int bathy_port;
  int water_column_port;
  int cmd_port;
  std::string sensor_frame;
  std::string pointcloud_topic;
  std::string bathymetric_topic;
  std::string detections_topic;
  std::string ranges_topic;
  std::string norbit_watercolumn_topic;
  std::string watercolumn_topic;
  double cmd_timeout;
  double disconnect_timeout;
  std::map<std::string, std::string> startup_settings;
  std::map<std::string, std::string> shutdown_settings;

  bool pub(std::string topic){
    return topic!="";
  }

  /*!
   * \brief do we need to publish a naieve projected pointcloud?
   */
  bool pubPointcloud(){
    return pub(pointcloud_topic);
  }

  /*!
   * \brief do we need to publish norbit_msgs batymeteric data?
   */
  bool pubBathymetric(){
    return pub(bathymetric_topic);
  }


  /*!
   * \brief do we need to publish acoustic_msgs multibeam detections data?
   */
  bool pubDetections() {
    return pub(detections_topic);
  }

  /*!
   * \brief do we need to publish norbit_msgs range data?
   */
  bool pubRanges() {
    return pub(ranges_topic);
  }


  /*!
   * \brief do we need to publish ANY Watercolum data?
   */
  bool pubWC(){
    return pub(norbit_watercolumn_topic) || pub(watercolumn_topic);
  }

  /*!
   * \brief do we need to publish norbit_msgs watercolum data?
   */
  bool pubNorbitWC(){
    return pub(norbit_watercolumn_topic);
  }

  /*!
   * \brief do we need to publish acoustic_msgs watercolumn data?
   */
  bool pubMultibeamWC(){
    return pub(watercolumn_topic);
  }
};