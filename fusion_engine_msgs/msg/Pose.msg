std_msgs/Header header
fusion_engine_msgs/Timestamp p1_time
fusion_engine_msgs/Timestamp gps_time

uint8  solution_type
int16  undulation

float64 latitude
float64 longitude
float64 altitude

float32[9] position_covariance

fusion_engine_msgs/RPY rpy
float32[9] rpy_covariance

geometry_msgs/Vector3 velocity_flu
float32[9] velocity_flu_covariance

float32 aggregate_protection_level
float32 horizontal_protection_level
float32 vertical_protection_level
