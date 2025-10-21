# fusion_engine_msgs/msg/PVTGeodetic.msg
# Copyright 2025 AI Racing Tech
#
# Septentrio SBF: PVTGeodetic (Block 4007)
# Parsed and published from raw SBF binary data.
# All units follow SI conventions.

std_msgs/Header header

# ----------------------------------------------------------------------
# Time (GPS)
# ----------------------------------------------------------------------
uint32 tow_ms                 # GPS Time of Week [ms]
uint16 week_number            # GPS Week Number
uint8  time_system            # Time system ID (0=GPS, 1=GLONASS, etc.)
uint8  datum                  # Datum ID

# ----------------------------------------------------------------------
# Position (geodetic)
# ----------------------------------------------------------------------
float64 latitude              # [deg]
float64 longitude             # [deg]
float64 height                # [m]

float32 undulation            # Geoid height (m)

# ----------------------------------------------------------------------
# Velocity (ENU)
# ----------------------------------------------------------------------
float32 north_velocity        # [m/s]
float32 east_velocity         # [m/s]
float32 up_velocity           # [m/s]
float32 course_over_ground    # [deg]

# ----------------------------------------------------------------------
# Clock & timing
# ----------------------------------------------------------------------
float32 rx_clock_bias         # [m]
float32 rx_clock_drift        # [m/s]
float32 latency               # [s]

# ----------------------------------------------------------------------
# Accuracy & corrections
# ----------------------------------------------------------------------
float32 horizontal_accuracy   # [m]
float32 vertical_accuracy     # [m]
float32 mean_corr_age         # [s]
uint16  reference_id
uint16  signal_info
uint16  ppp_info

# ----------------------------------------------------------------------
# Solution metadata
# ----------------------------------------------------------------------
uint8  mode
uint8  error
uint8  num_sats
uint8  wa_corr_info
uint8  alert_flag
uint8  num_bases
uint8  misc_flags
