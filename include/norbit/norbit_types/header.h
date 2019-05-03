#ifndef HEADER_H
#define HEADER_H

#include <iostream>
#include "norbit_definitions.h"

#pragma pack(push, 1)  // insert this to eliminiate automatic padding
namespace norbit_types {
  struct Header
  {
    uint32 preable;   //!< 0xDEADBEEF
    typeId type;      //!< Set to 1 for bathymetric data
    uint32 size;      //!< Size of entire package in bytes
    uint32 version;
    uint32 reserved;
    uint32 crc;       //!< CRC32 of the size-24 bytes following the header (crc32, zlib-style SEED=0x00000000, POLY=0xEDB88320)

    bool isValid(){return preable == NORBIT_PREAMBLE_KEY;}
    void print(){
      std::cout << "type:     " << type << std::endl;
      std::cout << "size:     " << size << std::endl;
      std::cout << "reserved: " << reserved << std::endl;
      std::cout << "crc:      " << crc << std::endl;
    }
  };
}
#pragma pack(pop)
#endif // HEADER_H
