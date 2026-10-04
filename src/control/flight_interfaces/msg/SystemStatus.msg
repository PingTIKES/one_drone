# Published alongside the legacy flight_state and flight_safety_status topics.
std_msgs/Header header
string flight_state
string vio_state
string hold_reason
bool px4_position_valid
bool px4_offboard
bool depth_valid
bool map_valid
bool trajectory_valid
