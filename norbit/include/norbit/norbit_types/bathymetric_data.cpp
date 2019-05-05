#include "bathymetric_data.h"
namespace norbit_types {
  BathymetricData::BathymetricData()
  {

  }

  void BathymetricData::setBits(std::shared_ptr<char> bits){
    bits_ = bits;
    header_ = reinterpret_cast<norbit_types::BathymetricData_H*>(
          bits_.get() );
    data_ = reinterpret_cast<norbit_types::BathymetricData_D*>(
          &bits_.get()[sizeof(norbit_types::BathymetricData_H)] );
  }
}
