std_msgs/Header header

# MAVLink ID of the vehicle, starts at 1
int32 mav_id

# reports the connection  status between MAVROS and FCU
bool connected

bool armed

# battery level in percentage
float32 battery

int32 flight_mode

int32 health