std_msgs/Header header

fusion_engine_msgs/Timestamp p1_time

uint8  solution_type
uint8  flags
uint8  mode

# Body-frame position std dev [Forward, Left, Up]
float32[3] position_std_body_m

# ENU position covariance (row-major, 3x3)
float64[9] position_covariance_enu_m2

# Quaternion (x,y,z,w): body w.r.t. ENU
float64[4] attitude_quaternion

# ENU velocities and std devs
float64[3] velocity_enu_mps
float32[3] velocity_std_enu_mps

# Flag bits
uint8 FLAG_VELOCITY_INCLUDES_TRACK=0x01
uint8 FLAG_WHEEL_SPEED_VALID=0x02
uint8 FLAG_WHEEL_TICKS_VALID=0x04

# Mode enum
uint8 MODE_FULL_NAV=0
uint8 MODE_GNSS_ONLY=1
uint8 MODE_INS_ONLY=2
