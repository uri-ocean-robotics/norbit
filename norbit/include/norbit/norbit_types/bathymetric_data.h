#ifndef BATHYMETRIC_DATA_H
#define BATHYMETRIC_DATA_H

#include "norbit_definitions.h"
#include <norbit_msgs/BathymetricStamped.h>

namespace norbit_types {

  class BathymetricData
  {
  public:
    BathymetricData();
    void setBits(std::shared_ptr<char> bits);
    norbit_msgs::BahtymetricHeader & getHeader(){return *header_;}
    norbit_msgs::BathymetricPoint & getData(size_t i){return data_[i];}
    norbit_msgs::BathymetricStamped getRosMsg(std::string frame_id);
  protected:
    norbit_msgs::BahtymetricHeader * header_;
    norbit_msgs::BathymetricPoint * data_;
    std::shared_ptr<char> bits_;
  };
}
#endif // BATHYMETRIC_DATA_H
