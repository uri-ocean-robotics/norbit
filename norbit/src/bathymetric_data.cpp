#include "norbit/norbit_types/bathymetric_data.h"
#include "norbit/ros_helper.hpp"

namespace norbit_types {
BathymetricData::BathymetricData()
{

}

void BathymetricData::setBits(std::shared_ptr<norbit_msgs::msg::CommonHeader> comm_hdr, std::shared_ptr<char> bits){
    bits_ = bits;
    comm_hdr_ = comm_hdr;
    bathymetric_header_ = reinterpret_cast<norbit_msgs::msg::BathymetricHeader*>(
          bits_.get() );
    data_ = reinterpret_cast<norbit_msgs::msg::BathymetricPoint*>(
          &bits_.get()[sizeof(norbit_msgs::msg::BathymetricHeader)] );
}

norbit_msgs::msg::BathymetricStamped BathymetricData::getRosMsg(std::string frame_id){
    norbit_msgs::msg::BathymetricStamped outMsg;
    outMsg.header.stamp = doubleToRosStamp(bathymetric_header_->time);
    outMsg.header.frame_id = frame_id;
    outMsg.bathy.common_header = *comm_hdr_;
    outMsg.bathy.bathymetric_header = *bathymetric_header_;
    outMsg.bathy.detections.assign(data_,data_+bathymetric_header_->beam_number);
    return outMsg;
}

}
