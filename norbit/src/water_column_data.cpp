#include "norbit/norbit_types/water_column_data.h"
#include "norbit/ros_helper.hpp"

namespace norbit_types {
WaterColumnData::WaterColumnData()
{

}

size_t WaterColumnData::dataSize(){
    size_t size = 0;
    switch (water_column_header_->dtype) {
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_UINT8:
            size=1;
            break;
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_INT8:
            size=1;
            break;
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_UINT16:
            size=2;
            break;
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_INT16:
            size=2;
            break;
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_UINT32:
            size=4;
            break;
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_INT32:
            size=4;
            break;
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_UINT64:
            size=8;
            break;
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_INT64:
            size=8;
            break;
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_FLOAT32:
            size=4;
            break;
        case norbit_msgs::msg::WaterColumnHeader::DTYPE_FLOAT64:
            size=8;
            break;
    }
    return size;
}

void WaterColumnData::setBits(std::shared_ptr<norbit_msgs::msg::CommonHeader> comm_hdr, std::shared_ptr<char> bits){
    bits_ = bits;
    comm_hdr_ = comm_hdr;
    water_column_header_ = reinterpret_cast<norbit_msgs::msg::WaterColumnHeader*>(
          bits_.get() );
    auto m = water_column_header_->sample_number;
    auto n = water_column_header_->beam_number;
    auto pixel_data_size = water_column_header_->sample_number * water_column_header_->beam_number * dataSize();
    auto pixel_data_offset = sizeof(norbit_msgs::msg::WaterColumnHeader);
    auto beam_directions_offset = pixel_data_offset+pixel_data_size;
    pixel_data_ = reinterpret_cast<uint8_t*>(
          &bits_.get()[pixel_data_offset] );
    beam_directions_ = reinterpret_cast<float32*>(
          &bits_.get()[beam_directions_offset]);
}


norbit_msgs::msg::WaterColumnStamped WaterColumnData::getRosMsg(std::string frame_id){
    auto m = water_column_header_->sample_number;
    auto n = water_column_header_->beam_number;

    norbit_msgs::msg::WaterColumnStamped outMsg;
    outMsg.header.stamp = doubleToRosStamp(water_column_header_->time);
    outMsg.header.frame_id = frame_id;
    outMsg.water_column.common_header = *comm_hdr_;
    outMsg.water_column.water_column_header = *water_column_header_;
    auto pixel_data_size = water_column_header_->sample_number * water_column_header_->beam_number * dataSize();
    outMsg.water_column.pixel_data.assign(pixel_data_,pixel_data_+pixel_data_size);
    outMsg.water_column.beam_directions.assign(beam_directions_,beam_directions_+n);
    return outMsg;
}
}
