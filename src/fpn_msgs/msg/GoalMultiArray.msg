# Custom message that includes a header, layout, an array of goals, and additional fields

std_msgs/Header header          # Include the standard header
std_msgs/MultiArrayLayout layout # Include MultiArrayLayout from std_msgs
fpn_msgs/Goal[] data           # Array of Goal messages
int32 goal_to_start_id         # ID for the goal to start
float64 zero_lat               # Latitude
float64 zero_lon               # Longitude
float32 zero_x                 # X-coordinate
float32 zero_y                 # Y-coordinate
