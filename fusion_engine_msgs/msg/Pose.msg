std_msgs/Header header

fusion_engine_msgs/Timestamp p1_time
fusion_engine_msgs/Timestamp gps_time

uint8  solution_type
uint8  flags
int16  undulation_cm

float64 latitude_deg
float64 longitude_deg
float64 altitude_m

# ENU position standard deviations [E, N, U]
float32[3] position_std_enu_m

# Euler-321, degrees [yaw, pitch, roll]
float64[3] ypr_deg
float32[3] ypr_std_deg

# Vehicle body-frame velocities [Forward, Left, Up]
float64[3] velocity_flu_mps
float32[3] velocity_std_flu_mps

float32 aggregate_protection_level_m
float32 horizontal_protection_level_m
float32 vertical_protection_level_m

# Flag bits (spec)
uint8 FLAG_STATIONARY=0x01
