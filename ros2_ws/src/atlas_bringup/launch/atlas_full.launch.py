"""
Atlas Full System Launch
========================
Launches all ROS 2 spine layers in order:
  1. Hardware (sensors + motors)
  2. Control (balance + gait)
  3. Perception (vision + SLAM)
  4. Planning (navigation)
  5. Supervisor (bridge to existing Atlas brain/ANS)

Usage:
  ros2 launch atlas_bringup atlas_full.launch.py
  ros2 launch atlas_bringup atlas_full.launch.py sim:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory("atlas_bringup")
    params_file = os.path.join(pkg_bringup, "config", "atlas_params.yaml")

    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument("sim", default_value="true",
                              description="Run in simulation mode (no real hardware)"),
        DeclareLaunchArgument("params_file", default_value=params_file,
                              description="Path to atlas_params.yaml"),

        # --- Layer 1: Hardware / Sensors ---
        Node(
            package="atlas_sensors",
            executable="imu_publisher",
            name="imu_publisher",
            parameters=[LaunchConfiguration("params_file")],
            condition=UnlessCondition(LaunchConfiguration("sim")),
        ),
        Node(
            package="atlas_sensors",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[LaunchConfiguration("params_file")],
        ),

        # --- Layer 2: Motor Control ---
        Node(
            package="atlas_motor_control",
            executable="balance_controller",
            name="balance_controller",
            parameters=[LaunchConfiguration("params_file")],
        ),
        Node(
            package="atlas_motor_control",
            executable="gait_generator",
            name="gait_generator",
            parameters=[LaunchConfiguration("params_file")],
        ),

        # --- Layer 3: Perception ---
        Node(
            package="atlas_perception",
            executable="vision_node",
            name="vision_node",
            parameters=[LaunchConfiguration("params_file")],
        ),
        Node(
            package="atlas_perception",
            executable="object_detector",
            name="object_detector",
            parameters=[LaunchConfiguration("params_file")],
        ),

        # --- Layer 4: Planning ---
        Node(
            package="atlas_planning",
            executable="task_planner",
            name="task_planner",
            parameters=[LaunchConfiguration("params_file")],
        ),
        Node(
            package="atlas_planning",
            executable="path_planner",
            name="path_planner",
            parameters=[LaunchConfiguration("params_file")],
        ),

        # --- Layer 5: Supervisor (bridge to existing Atlas) ---
        Node(
            package="atlas_supervisor",
            executable="supervisor_node",
            name="atlas_supervisor",
            parameters=[LaunchConfiguration("params_file")],
        ),
        Node(
            package="atlas_brain_bridge",
            executable="brain_bridge_node",
            name="brain_bridge",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
