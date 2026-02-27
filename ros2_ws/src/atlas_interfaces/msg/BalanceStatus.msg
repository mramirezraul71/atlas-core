# Balance controller status
std_msgs/Header header
string balance_mode                  # static | dynamic | impulse
float64 pitch_error                  # radians
float64 roll_error                   # radians
geometry_msgs/Vector3 cop            # Center of Pressure
geometry_msgs/Vector3 zmp            # Zero Moment Point
bool stable                          # true if within stability margin
float64 stability_margin             # distance from support polygon edge
