// Copyright 2025 AI Racing Tech
//
// SBF Binary Structures and Enumerations for Septentrio GNSS Devices
// Reference: Septentrio SBF Reference Guide
//
// All structs are packed and correspond directly to raw binary SBF payloads.
// Units and field names match SBF manual conventions.
// Extend as needed for additional SBF block IDs.

#ifndef FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_TYPEDEFS_HPP_
#define FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_TYPEDEFS_HPP_

#include <cstdint>
#include <stdint.h>
#include <stddef.h>
// ============================================================
// FusionEngine Input Data Types
// ============================================================

enum class InputDataType : uint16_t {
  UNKNOWN           = 0x0000,
  SBF_DATA          = 0x00B8,  // Septentrio Binary Format (observed PointOne data_type)
};

enum class SBFBlockID : uint16_t {
  // ------------------------------------------------------------
  // Core Measurement Blocks
  // ------------------------------------------------------------
  MeasEpoch         = 4027,
  MeasExtra         = 4000,
  EndOfMeas         = 5922,
  GPSNav            = 5891,
  GPSIonoParams     = 5893,
  GPSUtc            = 5894,
  GALTime           = 5896,
  IMUData           = 5930,
  IMUStatus         = 5931,

  // ------------------------------------------------------------
  // PVT (Position, Velocity, Time)
  // ------------------------------------------------------------
  PVTInfo           = 4003,
  PVTCartesian      = 4002,
  PVTCartesian_v2   = 4006,
  PVTGeodetic       = 4007,
  PosCovCartesian   = 5905,
  PosCovGeodetic    = 5906,
  VelCovCartesian   = 5907,
  VelCovGeodetic    = 5908,
  EndOfPVT          = 5921,
  GEOPRNMask        = 5926,

  // ------------------------------------------------------------
  // Attitude / INS Blocks
  // ------------------------------------------------------------
  AttEuler          = 5938,
  AttCovEuler       = 5939,
  AttCovMatrix      = 5897,
  AttEulerLegacy    = 5942,
  EndOfAtt          = 5943,
  INSNavCart        = 5932,
  INSNavGeod        = 5933,
  INSNavAtt         = 5934,
  IMUSetup          = 5929,
  ExtSensorMeas     = 5927,
  ExtSensorSetup    = 5928,

  // ------------------------------------------------------------
  // Receiver / Status Information
  // ------------------------------------------------------------
  ReceiverSetup     = 5902,
  xPPSOffset        = 5911,
  ReceiverTime      = 5914,
  DiffCorrIn        = 5919,
  ChannelStatus     = 4013,
  ReceiverStatus    = 4014,
  NTRIPClientStatus = 4053,

  // ------------------------------------------------------------
  // GNSS Raw Measurement Blocks
  // ------------------------------------------------------------
  DOP               = 4001,
  GEORawL1          = 4020,
  GPSRawL2C         = 4017,
  GPSRawL2P         = 4018,
  GALRawFNAV        = 4019,
  GALRawE5b         = 4022,
  GALRawINAV        = 4023,
  GLORawCA          = 4026,
  GLOTime           = 4036,
  QZSSRawL6         = 4030,
  QZSSRawL1CA       = 4031,
  BaseStationInfo   = 4032,
  BDSRaw            = 4047,
  BDSIon            = 4120,
  BDSRawB1C         = 4218,
  BDSRawB2a         = 4219,

  // ------------------------------------------------------------
  // Miscellaneous / Local Frames
  // ------------------------------------------------------------
  PosLocal          = 4052,
};

#pragma pack(push, 1)

struct PVTCartesian {
  uint32_t TOW;
  uint16_t WNc;
  uint8_t Mode;
  uint8_t Error;
  double x;
  double y;
  double z;
  float Undulation;
  float vx;
  float vy;
  float vz;
  float RxClkBias;
  float RxClkDrift;
  uint8_t TimeSystem;
  uint8_t Datum;
  uint8_t NrSV;
  uint8_t WACorrInfo;
};

// PVTCartesian_v2 (block 4006) uses the same structure as PVTCartesian (block 4002)
using PVTCartesian_v2 = PVTCartesian;

struct PVTGeodetic {
  uint32_t TOW;
  uint16_t WNc;
  uint8_t  Mode;
  uint8_t  Error;
  double   latitude;
  double   longitude;
  double   height;
  float    undulation;
  float    vn;
  float    ve;
  float    vu;
  float    cog;
  float    RxClkBias;
  float    RxClkDrift;
  uint8_t  timeSystem;
  uint8_t  datum;
  uint8_t  nrSV;
  uint8_t  WACorrInfo;
  uint16_t referenceID;
  uint16_t meanCorrAge;
  uint16_t signalInfo;
  uint8_t  alertFlag;
  uint8_t  NrBases;
  uint16_t PPPInfo;
  uint16_t latency;
  uint16_t HAccuracy;
  uint16_t VAccuracy;
  uint8_t  misc;
  uint8_t  padding[3];
};

#pragma pack(pop)

#endif  // FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_TYPEDEFS_HPP_
