std_msgs/Header header

fusion_engine_msgs/Timestamp p1_time
fusion_engine_msgs/Timestamp gps_time

uint8  solution_type

uint32 reference_station_id   # 0xFFFFFFFF if invalid

float64 east
float64 north
float64 up

float32[3] enu_covariance
