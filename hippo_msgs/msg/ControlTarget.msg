std_msgs/Header header

uint32 mask


uint32 IGNORE_POSITION_X = 1
uint32 IGNORE_POSITION_Y = 2
uint32 IGNORE_POSITION_Z = 4

uint32 IGNORE_POSITION = 7

uint32 IGNORE_ATTITUDE_X = 8
uint32 IGNORE_ATTITUDE_Y = 16
uint32 IGNORE_ATTITUDE_Z = 32
uint32 IGNORE_ATTITUDE = 56

uint32 IGNORE_POSE = 63

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

geometry_msgs/Quaternion attitude
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 angular_acceleration