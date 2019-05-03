#ifndef RAW_DETECTION_DATA_H
#define RAW_DETECTION_DATA_H
#include "s7k.h"
#pragma pack(push, 1)
namespace s7k {
  namespace RawDetectionData {
    ///
    /// \brief detection record header
    ///
    struct RTH
    {
      u64 sonarId;            //!<Sonar serial number
      u32 pingNumber;         //!<Sequential number
      u16 multipingSequence;  //!<Flag to indicate multiping sequence.Always 0 (zero) if not in multiping mode; otherwise this represents the sequence number of the ping in the multiping sequence.
      u32 n;                  //!<Number of detection points
      u32 dataFieldSize;      //!<Size of detection information block in bytes
      u8  detectionAlgorithm; //!<0 –G1_Simple1 –G1_BlendFilt2 –G23 –G34 –IF15 –PS1 (beam detection)6 –HS1 (beam detection)7 –HS2 (pseudo beam detection)8-255–Reserved for future use
      u32 flags;              //!<BIT FIELD:Bit 0-3:Uncertainty method0 –Not calculated1 –Rob Hare’s method2 –Ifremer’s method3-15 –Reserved for future useBit 4:Multi-detection enabledBit 5: ReservedBit 6: Has Snippetsdetection point flagBit 7-31:Reserved for future use
      f32 samplingRate;       //!<Sonar’s sampling frequency in Hz
      f32 txAngle;            //!<Applied transmitter steering angle, in radiansThis angle is used for pitch stabilization. It will be zero if the system doesn’t have this feature.The value is the same as the Projector beam steering angle of the 7000 record. They are both filled from the same variable.
      f32 appliedRoll;        //!<Roll value (in radians) applied to gates; zero if roll stabilization is ON.This value is made available to be able todraw the gating lines in the real-time user interface wedge display
      u32 reserved[15];      //!<Reserved for future use
    };
    ///
    /// \brief the description of each detection point
    ///
    struct RD
    {
      u16 beamDescriptor;   //!<Beam number the detection is taken from
      f32 detectionPoint;   //!<Non-corrected fractional sample number with reference to receiver’s acoustic center with the zero sample at the transmit time
      f32 rxAngle;          //!<Beam steering angle with reference to receiver’s acoustic center in the sonar reference frame, at the detection point; in radians
      u32 flags;            //!<BIT FIELD:Bit 0:1 –Intensity (Magnitude)based detectionBit 1:1 –Phase based detectionBit 2-8:Quality type, defines the type of the quality field belowBits 9-12: Detection priority number for detections withinthe same beam (Multi-detect only). Value zero ishighest priority.Bits 13:ReservedBit 14:Snippetdetection point flag0 –Detection used in snippet1 –Not used in snippetBits 15-31:Reserved for future use
      u32 quality;          //!<Detection quality
      f32 uncertainty;      //!<Detection uncertainty represented as an error normalized to the detection point
      f32 intensity;        //!<Intensity of detection point
      //f32 minLimit;         //!<Minimum sample number of gate limit
      //f32 maxLimit;         //!<Max limit
    };
    ///
    /// \brief no optional data for this record type
    ///
    struct OD
    {
      // none for this record
    };


    ///
    /// \brief This record is produced by the 7k sonar source series. It contains non-compensated detection results. The 7k sonar sourceupdates this record on every ping. This record is available by subscription only
    ///
    class Record
    {
    public:
      Record();
      void fromByteArray(char * array);
      const static unsigned int recordId = 2027;

    protected:
      RTH * rth;
      RD * rd;
      OD * od;
    };
  }
}
#pragma pack(pop)
#endif // RAW_DETECTION_DATA_H
