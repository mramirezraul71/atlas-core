# Full robot state snapshot
std_msgs/Header header
string robot_name
string mode                          # governed | growth | emergency
float64 battery_level                # 0.0 - 1.0
bool hardware_connected
bool brain_online
bool supervisor_active
float64 uptime_seconds
sensor_msgs/JointState joint_state
geometry_msgs/Vector3 center_of_mass
