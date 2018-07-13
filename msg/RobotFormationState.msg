std_msgs/Header header

# Robot id starts from 0
int32 robot_id

# if robot received its assigned goal location
bool received_goal

# True if robot received 'start' signal
bool mission_started

# if robot arrived to goal
bool arrived

geometry_msgs/Point point