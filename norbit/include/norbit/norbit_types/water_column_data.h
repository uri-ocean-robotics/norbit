#ifndef WATER_COLUMN_DATA_H
#define WATER_COLUMN_DATA_H

#include "norbit_definitions.h"
#include <norbit_msgs/WaterColumnStamped.h>

namespace norbit_types {

  class WaterColumnData
  {
  public:
    WaterColumnData();
    size_t dataSize();
    void setBits(std::shared_ptr<norbit_msgs::CommonHeader> comm_hdr, std::shared_ptr<char> bits);
//    norbit_msgs::BahtymetricHeader & bathymetricHeader(){return *bahtymetric_header_;}
//    norbit_msgs::BathymetricPoint & data(size_t i){return data_[i];}
    norbit_msgs::WaterColumnStamped getRosMsg(std::string frame_id);
  protected:
    std::shared_ptr<norbit_msgs::CommonHeader> comm_hdr_;

    std::shared_ptr<char> bits_;
    norbit_msgs::WaterColumnHeader * water_column_header_;
    uint8_t * pixel_data_;
    float32 * beam_directions_;
  };
}
#endif // BATHYMETRIC_DATA_H
