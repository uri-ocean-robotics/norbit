#include "norbit_types/bathymetric_data.h"
namespace norbit_types {
  BathymetricData::BathymetricData()
  {

  }

  void BathymetricData::setBits(std::shared_ptr<norbit_msgs::CommonHeader> comm_hdr, std::shared_ptr<char> bits){
    bits_ = bits;
    comm_hdr_ = comm_hdr;
    bathymetric_header_ = reinterpret_cast<norbit_msgs::BathymetricHeader*>(
          bits_.get() );
    data_ = reinterpret_cast<norbit_msgs::BathymetricPoint*>(
          &bits_.get()[sizeof(norbit_msgs::BathymetricHeader)] );
  }
  norbit_msgs::BathymetricStamped BathymetricData::getRosMsg(std::string frame_id){
    norbit_msgs::BathymetricStamped outMsg;
    ros::Time stamp(bathymetric_header_->time);
    outMsg.header.stamp = stamp;
    outMsg.header.frame_id = frame_id;
    outMsg.bathy.common_header = *comm_hdr_;
    outMsg.bathy.bathymetric_header = *bathymetric_header_;
    outMsg.bathy.detections.assign(data_,data_+bathymetric_header_->N);
    return outMsg;
  }
}
