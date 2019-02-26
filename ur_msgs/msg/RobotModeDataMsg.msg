# This data structure contains the RobotModeData structure
# used by the Universal Robots controller
#
# This data structure is send at 10 Hz on TCP port 30002
#
# Note: this message does not carry all fields from the RobotModeData structure as broadcast by the robot controller, but a subset.

uint64 timestamp
bool is_robot_connected
bool is_real_robot_enabled
bool is_power_on_robot
bool is_emergency_stopped
bool is_protective_stopped
bool is_program_running
bool is_program_paused
