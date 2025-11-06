# fusion_engine_msgs/msg/VelCovGeodetic.msg
# Copyright 2025 AI Racing Tech
#
# Septentrio SBF: VelCovGeodetic (Block XXXX)
# Velocity covariance matrix in geodetic frame (North, East, Up).

std_msgs/Header header

# ----------------------------------------------------------------------
# Time (GPS)
# ----------------------------------------------------------------------
uint32 tow_ms        # GPS Time of Week [ms]
uint16 week_number   # GPS Week Number

# ----------------------------------------------------------------------
# Velocity covariance ((m/s)^2)
# ----------------------------------------------------------------------
float32 cov_vn_vn    # Variance Vn-Vn
float32 cov_vn_ve    # Covariance Vn-Ve
float32 cov_vn_vu    # Covariance Vn-Vu
float32 cov_ve_ve    # Variance Ve-Ve
float32 cov_ve_vu    # Covariance Ve-Vu
float32 cov_vu_vu    # Variance Vu-Vu
