#include "norbit_types/water_column_data.h"
namespace norbit_types {
  WaterColumnData::WaterColumnData()
  {

  }

  size_t WaterColumnData::dataSize(){
    size_t size = 0;
    switch (water_column_header_->sonar_mode) {
      case norbit_msgs::WaterColumnHeader::DTYPE_UINT8:
        size=1;
        break;
      case norbit_msgs::WaterColumnHeader::DTYPE_INT8:
        size=1;
        break;
      case norbit_msgs::WaterColumnHeader::DTYPE_INT16:
        size=2;
        break;
      case norbit_msgs::WaterColumnHeader::DTYPE_UINT32:
        size=4;
        break;
      case norbit_msgs::WaterColumnHeader::DTYPE_INT32:
        size=4;
        break;
      case norbit_msgs::WaterColumnHeader::DTYPE_UINT64:
        size=8;
        break;
      case norbit_msgs::WaterColumnHeader::DTYPE_INT64:
        size=8;
        break;
      case norbit_msgs::WaterColumnHeader::DTYPE_FLOAT32:
        size=4;
        break;
      case norbit_msgs::WaterColumnHeader::DTYPE_FLOAT64:
        size=8;
        break;
    }
    return size;
  }

  void WaterColumnData::setBits(std::shared_ptr<norbit_msgs::CommonHeader> comm_hdr, std::shared_ptr<char> bits){
    bits_ = bits;
    comm_hdr_ = comm_hdr;
    water_column_header_ = reinterpret_cast<norbit_msgs::WaterColumnHeader*>(
          bits_.get() );
    auto m = water_column_header_->M;
    auto n = water_column_header_->N;
    auto dtype_size = water_column_header_->M * water_column_header_->N * dataSize();
    pixel_data_ = reinterpret_cast<uint8_t*>(
          &bits_.get()[sizeof(norbit_msgs::WaterColumnHeader)] );
    beam_directions_ = reinterpret_cast<float32*>(
          &bits_.get()[dtype_size*m*n]);
  }


  norbit_msgs::WaterColumnStamped WaterColumnData::getRosMsg(std::string frame_id){
    auto m = water_column_header_->M;
    auto n = water_column_header_->N;

    norbit_msgs::WaterColumnStamped outMsg;
    ros::Time stamp(water_column_header_->time);
    outMsg.header.stamp = stamp;
    outMsg.header.frame_id = frame_id;
    outMsg.water_column.common_header = *comm_hdr_;
    outMsg.water_column.water_column_header = *water_column_header_;
    outMsg.water_column.pixel_data.assign(pixel_data_,pixel_data_+m*n);
    outMsg.water_column.beam_directions.assign(beam_directions_,beam_directions_+n);
    return outMsg;
  }
}
