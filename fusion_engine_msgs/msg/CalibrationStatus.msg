std_msgs/Header header

fusion_engine_msgs/Timestamp p1_time

uint8  calibration_stage

float32[3] ypr_mounting_angle
float32[3] ypr_std_dev_deg
float32[3] max_ypr_std_dev_deg

float32 travel_distance_m
float32 min_travel_distance_m

uint8  state_verified            # 1=verified, 0 otherwise

uint8  gyro_bias_percent         # units of 0.5% (0..200)
uint8  accel_bias_percent        # units of 0.5% (0..200)
uint8  mounting_angle_bias_percent # units of 0.5% (0..200)

# CalibrationStage (u8)
uint8 CAL_STAGE_UNKNOWN=0x00
uint8 CAL_STAGE_MOUNTING_ANGLE=0x01
uint8 CAL_STAGE_DONE=0xFF
