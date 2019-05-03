#ifndef BATHYMETRIC_DATA_H
#define BATHYMETRIC_DATA_H

#include "norbit_definitions.h"

namespace norbit_types {

  enum sonar_mode_t: uint8{
    fls = 0,
    bathy = 1,
    bathy_amp = 2,
    bf_passthrough = 4,
    bathy_edge_enhance = 5,
    super_res_bathy = 6
  };

  struct BathymetricData_H
  {
    float32 snd_velocity;     //!< Sound velocity in m/s
    float32 sample_rate;      //!< Sampling rate in Hz
    uint32  N;                //!< Number of beams
    uint32  ping_number;      //!< Sequence number of ping
    float64 time;             //!< Ping timestamp, unix time at TX
    float64 time_net;         //!< Timestamp Unix time as fract (send on network time)
    float32 ping_rate;        //!< In Hz
    uint16  type;             //!< Bathy data type (4)
    uint8   beam_dist_mode;   //!< 1:512EA 2:256EA
    sonar_mode_t   sonar_mode;//!< see: sonar_mode_t
    uint16  reserved1[4];
    float32 tx_angle;         //!< Tx elevation steering in radians
    float32 gain;             //!< Intensity value gain
    float32 tx_freq;          //!< Tx frequency in Hz
    float32 tx_bw;            //!< Tx bandwidth in Hz
    float32 tx_len;           //!< Tx pulse length in sec
    float32 reserved2;
    float32 tx_voltage;       //!< Tx peak-voltage over elements, NaN on non-STX sonars.
    float32 swath_dir;        //!< Center pointing direction of swath, in radians
    float32 swath_open;       //!< Opening angle of swath, in radians
    float32 gate_tilt;        //!< Gate tilt in radians, for depth gates
  };

  struct BathymetricData_D
  {
    uint32  sample_number;    //!< Sample number for first beam
    float32 angle;            //!< Angle in radians for first beam
    uint16  upper_gate;       //!< Sample number for upper adaptive gate
    uint16  lower_gate;       //!< Sample number for lower adaptive gate
    uint32  intensity;        //!< Intensity for first beam
    uint16  flags;            //!< Info flags for first beam
    uint8   quality_flag;     //!< Quality flags for first beam
    uint8   quality_val;      //!< Quality value for first beam
  };

  class BathymetricData
  {
  public:
    BathymetricData();
    void setBits(std::shared_ptr<char> bits);
    BathymetricData_H & getHeader(){return *header_;}
    BathymetricData_D & getData(size_t i){return data_[i];}
  protected:
    BathymetricData_H * header_;
    BathymetricData_D * data_;
    std::shared_ptr<char> bits_;
  };
}
#endif // BATHYMETRIC_DATA_H
