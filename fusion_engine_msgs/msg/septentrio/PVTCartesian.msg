# fusion_engine_msgs/msg/PVTCartesian.msg
# Copyright 2025 AI Racing Tech
#
# Septentrio SBF: PVTCartesian (Block 4006)
# Cartesian position, velocity, and timing information.

std_msgs/Header header

# ----------------------------------------------------------------------
# Time (GPS)
# ----------------------------------------------------------------------
uint32 tow_ms         # GPS Time of Week [ms]
uint16 week_number    # GPS Week Number

# ----------------------------------------------------------------------
# Solution metadata
# ----------------------------------------------------------------------
uint8  mode           # Position mode
uint8  error          # Error status flag

# ----------------------------------------------------------------------
# Position (ECEF)
# ----------------------------------------------------------------------
float64 x             # X coordinate [m]
float64 y             # Y coordinate [m]
float64 z             # Z coordinate [m]
float32 undulation    # Geoid height (m)

# ----------------------------------------------------------------------
# Velocity (ECEF)
# ----------------------------------------------------------------------
float32 vx            # Velocity X [m/s]
float32 vy            # Velocity Y [m/s]
float32 vz            # Velocity Z [m/s]

# ----------------------------------------------------------------------
# Clock & timing
# ----------------------------------------------------------------------
float32 rx_clock_bias   # Receiver clock bias [m]
float32 rx_clock_drift  # Receiver clock drift [m/s]

# ----------------------------------------------------------------------
# Reference & satellite info
# ----------------------------------------------------------------------
uint8  time_system     # Time system ID (0=GPS, 1=GLONASS, etc.)
uint8  datum           # Datum ID
uint8  num_sats        # Number of satellites used
uint8  wa_corr_info    # WAAS/EGNOS correction info
