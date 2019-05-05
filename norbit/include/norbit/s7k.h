#ifndef S7K_H
#define S7K_H

#include <cstdint>
#include <cstring>

namespace s7k {


// define all the datypes in the ICD as their equivalent c++ types
using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

using f32 = float;


#pragma pack(push, 1)  // insert this to eliminiate automatic padding
/*!
 * \brief The DataRecordFrameHead struct
 *
 * from: https://www.teledyne-pds.com/wp-content/uploads/2016/11/DATA-FORMAT-DEFINITION-DOCUMENT-7k-Data-Format-Volume-I-Version-3.01.pdf
 *
 * The Data Record Frame (DRF) is the wrapper in which all records
 * (sensor data or otherwise) shall be embedded. The sync pattern
 * combined with the checksum should aid recovery in the event a
 * file becomes corrupted. A record frame shall always start with
 * the version and offset fields and can be used to dynamically
 * determine the protocol version, if necessary.
 */
struct DataRecordFrameHead{
  u16 protocolVersion;  //!< Protocol version of this frame
  u16 offset;          //!< Offset in bytes from the start of the sync pattern to the start of the Record Type Header (RTH). This allows for expansion of the header whilst maintaining backward compatibility.
  u32 syncPattern;     //!< 0x0000FFFF
  u32 size;            //!< Size in bytes of this record from the start of the Protocol version field to the end of the checksum field —including any embedded data
  u32 optionalDataOffset; //!< Offset in bytes to optional data field from start of record. Zero (0) bytes implies no optional data.
  u32 optionalDataIdentifier;
  u8  s7kTime[10];     //!< Time tag indicating when data was produced
  u16 recordVersion;   //!< Currently 1
  u32 recordTypeIdentifier; //!< Identifier for record type of embedded data
  u32 deviceIdentifier;//!< Identifier of the device to which this data pertains
  u16 reserved1;       //!< reserved
  u16 systemEnumerator;//!< The enumerator is used to differentiate between devices with the same device identifiers in one installation/system. For example, on 7125 200khz/400kHz dual-frequency systems, the enumerator will normally be zero (0) in 200khz mode, and  one (1) in 400kHz mode
  u32 reserved2;       //!< reserved
  u16 flags;           //!< BIT FIELD:Bit 0:Checksum0 –Invalid checksum1 –Valid checksumBit 1-14:Reserved (must be zero)Bit 15:0 –Live data1 –Recorded data
  u16 reserved3;
  u32 reserved4;
  u32 totalRecordsInFragmentedData; //!< Always zero
  u32 fragmentNumber;               //!< Always zero
};

struct DataRecordFrameTail{
  u32 checksum;       //!<  Sum of all byte values (treated as unsigned) in the record from the beginning of the version field to the end of the data section. The use of this field is optional and depends on bit 1 of the Flags field. The checksum should be computed as a 32 bit unsigned integer.
};

struct NetworkFrame{
  u16 protocolVersion;
  u16 offset;
  u32 totalPackets;
  u16 totalRecords;
  u16 transmissionIdentifier;
  u32 packetSize;
  u32 totalSize;
  u32 sequenceNumber;
  u32 destinationDeviceIdentifier;
  u16 DestinationEnumerator;
  u16 sourceEnumerator;
  u32 sourceDeviceIdentifier;
};


class S7K{
public:
  S7K();

};

struct S7K_t{
  const DataRecordFrameHead * DRF;
  const DataRecordFrameTail * DRFTail;
};

#pragma pack(pop)
}

#endif // S7K_H
