std_msgs/Header header

uint32 mask
uint32 IGNORE_ROLL_ANGLE = 1
uint32 IGNORE_PITCH_ANGLE = 2
uint32 IGNORE_YAW_ANGLE = 4

uint32 IGNORE_ROLL_RATE = 8
uint32 IGNORE_PITCH_RATE = 16
uint32 IGNORE_YAW_RATE = 32

uint32 IGNORE_THRUST = 64

uint32 IGNORE_RATES = 56
uint32 IGNORE_ATTITUDE = 7

float64 thrust
geometry_msgs/Quaternion attitude
geometry_msgs/Vector3 body_rate
