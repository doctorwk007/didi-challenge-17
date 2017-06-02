# Obstacle details: id, class, classification score, size and position (2D and 3D), rotation, velocity and more
Header header

uint32 id
string kind_name
int8 kind
float32 score

int8 truncated
int8 occluded

int8 disp_min
int8 disp_max
int8 elevated

float32 height
float32 width
float32 length
sensor_msgs/RegionOfInterest bbox
geometry_msgs/Point32 location
float32 alpha
float32 yaw
geometry_msgs/Twist velocity
