std_msgs/Header header
# Effort in [-1, 1], body FLU; fresh stamp and frame_id=base_link required.
# The vehicle adapter alone maps effort to hardware-specific PWM.
bool arm
string mode
float32 forward
float32 lateral
float32 thrust
float32 pitch
float32 roll
float32 yaw
