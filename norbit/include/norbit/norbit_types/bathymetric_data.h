#ifndef BATHYMETRIC_DATA_H
#define BATHYMETRIC_DATA_H

#include "norbit_definitions.h"
#include <norbit_msgs/BathymetricStamped.h>

namespace norbit_types {

  class BathymetricData
  {
  public:
    BathymetricData();
    void setBits(std::shared_ptr<norbit_msgs::CommonHeader> comm_hdr, std::shared_ptr<char> bits);
    norbit_msgs::BathymetricHeader & bathymetricHeader(){return *bathymetric_header_;}
    norbit_msgs::BathymetricPoint & data(size_t i){return data_[i];}
    norbit_msgs::BathymetricStamped getRosMsg(std::string frame_id);
  protected:
    std::shared_ptr<norbit_msgs::CommonHeader> comm_hdr_;

    std::shared_ptr<char> bits_;
    norbit_msgs::BathymetricHeader * bathymetric_header_;
    norbit_msgs::BathymetricPoint * data_;

  };
}
#endif // BATHYMETRIC_DATA_H
