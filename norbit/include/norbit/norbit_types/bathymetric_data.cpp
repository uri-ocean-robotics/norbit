#include "bathymetric_data.h"
namespace norbit_types {
  BathymetricData::BathymetricData()
  {

  }

  void BathymetricData::setBits(std::shared_ptr<char> bits){
    bits_ = bits;
    header_ = reinterpret_cast<norbit_msgs::BahtymetricHeader*>(
          bits_.get() );
    data_ = reinterpret_cast<norbit_msgs::BathymetricPoint*>(
          &bits_.get()[sizeof(norbit_msgs::BahtymetricHeader)] );
  }
  norbit_msgs::BathymetricStamped BathymetricData::getRosMsg(std::string frame_id){
    norbit_msgs::BathymetricStamped outMsg;
    ros::Time stamp(header_->time);
    outMsg.header.stamp = stamp;
    outMsg.header.frame_id = frame_id;
    outMsg.bathy.header = *header_;
    outMsg.bathy.detections.assign(data_,data_+header_->N);
    return outMsg;
  }
}
