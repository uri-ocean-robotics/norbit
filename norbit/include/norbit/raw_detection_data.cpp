#include "raw_detection_data.h"
namespace s7k {
  namespace RawDetectionData {
    Record::Record()
    {

    }

    void Record::fromByteArray(char *array){
      rth = reinterpret_cast<s7k::RawDetectionData::RTH*>(
            array );
      rd  = reinterpret_cast<s7k::RawDetectionData::RD*>(
            array[sizeof(s7k::RawDetectionData::RTH)]);
    }
  }

}
