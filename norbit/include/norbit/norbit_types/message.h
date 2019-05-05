#ifndef MESSAGE_H
#define MESSAGE_H

#include <iostream>
#include <cstring>
#include "header.h"
#include "bathymetric_data.h"
namespace norbit_types {
class Message
  {
  public:
    Message(){return;}
    bool fromBoostArray(boost::array<char, sizeof(norbit_types::Header)> array){
      header_.reset(new Header);
      memcpy(header_.get(),&array,sizeof(Header));
      return isValid();
    }
    Header getHeader(){return *header_;}
    bool isValid(){return header_->isValid();}
    void setBits(std::shared_ptr<char> bits){
      switch (header_->type) {
      case bathy:
          bathy_.setBits(bits);
        break;
      default:
          std::cerr << "unable to read data:  unknown/undefined type"  << std::endl;
        break;
      }
    }
    BathymetricData getBathy(){return bathy_;}

  protected:
    std::shared_ptr<Header> header_;
    BathymetricData bathy_;

  };
}
#endif // MESSAGE_H
