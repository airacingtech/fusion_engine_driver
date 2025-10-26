std_msgs/Header header
fusion_engine_msgs/Timestamp p1_time
uint8  calibration_stage
uint8  state_verified

fusion_engine_msgs/RPY rpy_mounting
float32[3] rpy_mounting_covariance
float32[3] rpy_max_covariance
float32[3] max_ypr_std_dev_deg

float32 travel_distance
float32 min_travel_distance

uint8  gyro_bias_percent
uint8  accel_bias_percent
uint8  mounting_angle_bias_percent