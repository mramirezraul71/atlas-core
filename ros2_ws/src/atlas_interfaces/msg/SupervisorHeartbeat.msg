# Heartbeat from supervisor to all subsystems
std_msgs/Header header
string governance_mode               # governed | growth | emergency
bool push_api_reachable              # can reach PUSH backend (8791)
bool robot_api_reachable             # can reach ROBOT backend (8002)
string[] active_tasks                # list of active task IDs
int32 pending_approvals
float64 system_health_score          # 0-100
