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

// ============================================================
// FusionEngine Input Data Types
// ============================================================

enum class InputDataType : uint16_t {
  UNKNOWN           = 0x0000,
  RTCM_DATA         = 0x0091,
  SPARTN_DATA       = 0x0092,
  SBF_DATA          = 0x00A1,  // Septentrio Binary Format
  NMEA_DATA         = 0x00A2,
  RAW_IMU_DATA      = 0x00B0,
  CUSTOM_USER_DATA  = 0x00C0,
};

// ============================================================
// Septentrio SBF Block Identifiers
// ============================================================

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

// ============================================================
// SBF Common Header
// ============================================================

#pragma pack(push, 1)

struct Header {
  uint16_t sync;    // 0x24 0x40
  uint16_t crc;     // CRC of full block
  uint16_t id;      // Block ID + Revision bits
  uint16_t length;  // Total length (bytes)
};

// ============================================================
// Core PVT blocks
// ============================================================

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

// ============================================================
// Covariance blocks
// ============================================================

struct PosCovGeodetic {
  uint32_t TOW;
  uint16_t WNc;
  float Cov_NN;
  float Cov_NE;
  float Cov_NU;
  float Cov_EE;
  float Cov_EU;
  float Cov_UU;
};

struct VelCovGeodetic {
  uint32_t TOW;
  uint16_t WNc;
  float Cov_VnVn;
  float Cov_VnVe;
  float Cov_VnVu;
  float Cov_VeVe;
  float Cov_VeVu;
  float Cov_VuVu;
};

// ============================================================
// Attitude blocks
// ============================================================

struct AttEuler {
  uint32_t TOW;
  uint16_t WNc;
  float Pitch_deg;
  float Roll_deg;
  float Heading_deg;
  uint8_t Mode;
  uint8_t Error;
  uint8_t NbSV;
  uint8_t Reserved;
};

struct AttCovEuler {
  uint32_t TOW;
  uint16_t WNc;
  float Cov_PitchPitch;
  float Cov_PitchRoll;
  float Cov_PitchHeading;
  float Cov_RollRoll;
  float Cov_RollHeading;
  float Cov_HeadingHeading;
};

// ============================================================
// INS Navigation blocks
// ============================================================

struct INSNavGeod {
  uint32_t TOW;
  uint16_t WNc;
  double Lat_deg;
  double Lon_deg;
  double Height_m;
  float Vn;
  float Ve;
  float Vu;
  float Roll_deg;
  float Pitch_deg;
  float Heading_deg;
  uint8_t INSStatus;
  uint8_t Error;
  uint16_t Reserved;
};

struct INSNavCart {
  uint32_t TOW;
  uint16_t WNc;
  double X;
  double Y;
  double Z;
  float Vx;
  float Vy;
  float Vz;
  float Roll_deg;
  float Pitch_deg;
  float Heading_deg;
  uint8_t INSStatus;
  uint8_t Error;
  uint16_t Reserved;
};

// ============================================================
// IMU / sensor configuration
// ============================================================

struct IMUSetup {
  uint32_t TOW;
  uint16_t WNc;
  uint8_t IMUType;
  uint8_t Reserved;
  float LeverArmX_m;
  float LeverArmY_m;
  float LeverArmZ_m;
  float RollOffset_deg;
  float PitchOffset_deg;
  float HeadingOffset_deg;
};

struct IMUData {
  uint32_t TOW;
  uint16_t WNc;
  int32_t DeltaVelX;
  int32_t DeltaVelY;
  int32_t DeltaVelZ;
  int32_t DeltaAngleX;
  int32_t DeltaAngleY;
  int32_t DeltaAngleZ;
  uint8_t IMUStatus;
  uint8_t Reserved;
};

// ============================================================
// Receiver time block
// ============================================================

struct ReceiverTime {
  uint32_t TOW;
  uint16_t WNc;
  int32_t ClockBias_ns;
  int32_t ClockDrift_ppb;
  uint8_t TimeSystem;
  uint8_t SyncLevel;
  uint16_t Reserved;
};

// ============================================================
// End-of-measurement / housekeeping
// ============================================================

struct EndOfPVT {
  uint32_t TOW;
  uint16_t WNc;
  uint16_t Reserved;
};

#pragma pack(pop)

#endif  // FUSION_ENGINE_DRIVER__COMMUNICATION__SBF_TYPEDEFS_HPP_
