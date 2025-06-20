#ifndef BATHYMETRIC_DATA_H
#define BATHYMETRIC_DATA_H

#include "norbit_definitions.h"
#include <norbit_msgs/msg/bathymetric_stamped.hpp>

namespace norbit_types {

class BathymetricData
{
public:
    BathymetricData();
    void setBits(std::shared_ptr<norbit_msgs::msg::CommonHeader> comm_hdr, std::shared_ptr<char> bits);
    norbit_msgs::msg::BathymetricHeader & bathymetricHeader(){return *bathymetric_header_;}
    norbit_msgs::msg::BathymetricPoint & data(size_t i){return data_[i];}
    norbit_msgs::msg::BathymetricStamped getRosMsg(std::string frame_id);
protected:
    std::shared_ptr<norbit_msgs::msg::CommonHeader> comm_hdr_;

    std::shared_ptr<char> bits_;
    norbit_msgs::msg::BathymetricHeader * bathymetric_header_;
    norbit_msgs::msg::BathymetricPoint * data_;
};
}
#endif // BATHYMETRIC_DATA_H
