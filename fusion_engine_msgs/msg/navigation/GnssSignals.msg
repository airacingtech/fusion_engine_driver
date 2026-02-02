std_msgs/Header header

# Point One timestamps
fusion_engine_msgs/Timestamp p1_time
fusion_engine_msgs/Timestamp gps_time

# GPS time info
uint32 gps_tow_ms      # INVALID = 0xFFFFFFFF
uint16 gps_week        # INVALID = 0xFFFF

# Counts
uint16 num_signals
uint8  num_satellites

# ---- Trailing variable-length data ----
# Order MUST match FusionEngine serialization:
# 1) all satellites
# 2) all signals

fusion_engine_msgs/GnssSatelliteInfo[] satellites
fusion_engine_msgs/GnssSignalInfo[] signals
