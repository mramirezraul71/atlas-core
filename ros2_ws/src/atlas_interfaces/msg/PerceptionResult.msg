# Perception pipeline output
std_msgs/Header header
string source                        # head_rgb | chest_depth | lidar
string[] detected_classes            # e.g. ["person", "chair", "door"]
float64[] confidences                # parallel to detected_classes
geometry_msgs/Point[] positions_3d   # 3D positions of detections
bool slam_active
geometry_msgs/Pose robot_pose_slam   # robot pose from SLAM (if active)
