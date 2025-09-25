std_msgs/Header header

fusion_engine_msgs/Timestamp p1_time
fusion_engine_msgs/Timestamp gps_time

uint8  leap_second           # 0xFF if invalid
uint8  num_satellites

uint16 corrections_age_decisec   # units of 0.1 s
uint16 baseline_distance_m     # units of 10 m

uint32 reference_station_id      # 0xFFFFFFFF if invalid

float32 gdop
float32 pdop
float32 hdop
float32 vdop
float32 gps_time_std_dev_s
