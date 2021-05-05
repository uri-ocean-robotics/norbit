#ifndef NORBIT_TYPEDEF_H
#define NORBIT_TYPEDEF_H

#include <stdint.h>
#include <memory>
#include <boost/array.hpp>


//#define NORBIT_PREAMBLE_KEY 0xDEADBEEF
#define NORBIT_CURRENT_VERSION 4

namespace norbit_types {
  // define all the datypes in the DFD as their equivalent c++ types
  using uint8  = uint8_t;
  using uint16 = uint16_t;
  using uint32 = uint32_t;
  using uint64 = uint64_t;

  using float32 = float;
  using float64 = double;

  enum typeId: uint32{
    bathymetric = 1,
    watercolum = 2,
    snippet = 4
  };
}


#endif // NORBIT_TYPEDEF_H
