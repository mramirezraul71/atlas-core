# High-level task goal from supervisor/brain
string task_id
string goal_description              # natural language goal
string mode                          # plan_only | controlled | auto
int32 priority                       # 1 (low) - 5 (critical)
string[] required_capabilities       # e.g. ["navigation", "manipulation", "vision"]
float64 timeout_seconds              # max execution time
