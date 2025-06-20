#ifndef WATER_COLUMN_DATA_H
#define WATER_COLUMN_DATA_H

#include "norbit_definitions.h"
#include <norbit_msgs/msg/common_header.hpp>
#include <norbit_msgs/msg/water_column_stamped.hpp>

namespace norbit_types {

class WaterColumnData
{
public:
    WaterColumnData();
    size_t dataSize();
    void setBits(std::shared_ptr<norbit_msgs::msg::CommonHeader> comm_hdr, std::shared_ptr<char> bits);
//    norbit_msgs::msg::BathymetricHeader & bathymetricHeader(){return *bathymetric_header_;}
//    norbit_msgs::msg::BathymetricPoint & data(size_t i){return data_[i];}
    norbit_msgs::msg::WaterColumnStamped getRosMsg(std::string frame_id);
protected:
    std::shared_ptr<norbit_msgs::msg::CommonHeader> comm_hdr_;

    std::shared_ptr<char> bits_;
    norbit_msgs::msg::WaterColumnHeader * water_column_header_;
    uint8_t * pixel_data_;
    float32 * beam_directions_;
};
}
#endif // BATHYMETRIC_DATA_H
