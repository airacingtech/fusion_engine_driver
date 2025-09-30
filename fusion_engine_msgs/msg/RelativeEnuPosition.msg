std_msgs/Header header

fusion_engine_msgs/Timestamp p1_time
fusion_engine_msgs/Timestamp gps_time

uint8  solution_type

uint32 reference_station_id   # 0xFFFFFFFF if invalid

float64 east_m
float64 north_m
float64 up_m

float32 east_std_m
float32 north_std_m
float32 up_std_m
