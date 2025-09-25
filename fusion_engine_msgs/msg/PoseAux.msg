std_msgs/Header header

fusion_engine_msgs/Timestamp p1_time

# Body-frame position std dev [Forward, Left, Up]
float32[3] position_std_body_m

# ENU position covariance (row-major, 3x3)
float64[9] position_covariance_enu_m2

# Quaternion (x,y,z,w): body w.r.t. ENU
float64[4] attitude_quaternion

# ENU velocities and std devs
float64[3] velocity_enu_mps
float32[3] velocity_std_enu_mps