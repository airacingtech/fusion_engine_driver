# fusion_engine_msgs/msg/PosCovGeodetic.msg
# Copyright 2025 AI Racing Tech
#
# Septentrio SBF: PosCovGeodetic (Block XXXX)
# Position covariance matrix in geodetic coordinates (North, East, Up).

std_msgs/Header header

# ----------------------------------------------------------------------
# Time (GPS)
# ----------------------------------------------------------------------
uint32 tow_ms        # GPS Time of Week [ms]
uint16 week_number   # GPS Week Number

# ----------------------------------------------------------------------
# Position covariance (m^2)
# ----------------------------------------------------------------------
float32 cov_nn       # Variance North-North
float32 cov_ne       # Covariance North-East
float32 cov_nu       # Covariance North-Up
float32 cov_ee       # Variance East-East
float32 cov_eu       # Covariance East-Up
float32 cov_uu       # Variance Up-Up
