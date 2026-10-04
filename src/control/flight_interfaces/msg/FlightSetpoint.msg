# Command selected by the supervisor and sent by the sole PX4 adapter.
std_msgs/Header header
uint8 MODE_POSITION=1
uint8 MODE_VELOCITY=2
uint8 mode
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float32 yaw
float32 yaw_rate
uint32 trajectory_id
float32 confidence
