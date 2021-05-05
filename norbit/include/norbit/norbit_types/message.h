#ifndef MESSAGE_H
#define MESSAGE_H

#include <iostream>
#include <cstring>
//#include "header.h"
#include "bathymetric_data.h"
#include <norbit_msgs/CommonHeader.h>
#include "CRC.h"
namespace norbit_types {
class Message
  {
  public:
    Message(){return;}
    bool fromBoostArray(boost::array<char, sizeof(norbit_msgs::CommonHeader)> array){
      header_.reset(new norbit_msgs::CommonHeader);
      memcpy(header_.get(),&array,sizeof(norbit_msgs::CommonHeader));
      return headerValid();
    }
    norbit_msgs::CommonHeader commonHeader(){return *header_;}
    bool headerValid(){
      return header_->preable==norbit_msgs::CommonHeader::NORBIT_PREAMBLE_KEY &&
             header_->version==NORBIT_CURRENT_VERSION;
    }
    bool setBits(std::shared_ptr<char> bits){
      bits_ = bits;
      if(msgValid()){
        switch (header_->type) {
        case bathymetric:
            bathy_.setBits(header_,bits);
          break;
        default:
            std::cerr << "unable to read data:  unknown/undefined type"  << std::endl;
          break;
        }
        return  true;
      }else {
        return false;
      }
    }
    bool msgValid(){
      std::uint32_t crc = CRC::Calculate(bits_.get(), header_->size - sizeof(norbit_msgs::CommonHeader) , CRC::CRC_32());
      return  crc==header_->crc;
    }
    BathymetricData getBathy(){return bathy_;}

  protected:
    std::shared_ptr<char> bits_;
    std::shared_ptr<norbit_msgs::CommonHeader> header_;
    BathymetricData bathy_;

  };
}
#endif // MESSAGE_H
