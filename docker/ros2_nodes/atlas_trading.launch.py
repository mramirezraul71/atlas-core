"""ROS2 Launch file para ATLAS Trading Node.

Uso:
    ros2 launch atlas_code_quant/docker/ros2_nodes/atlas_trading.launch.py

Requiere ROS2 Humble instalado en el Jetson.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Nivel de log: debug|info|warn|error",
        ),

        # Nodo principal de trading
        Node(
            package="atlas_code_quant",
            executable="atlas_trading_node",
            name="atlas_trading_node",
            output="screen",
            parameters=["atlas_trading_node.yaml"],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[
                ("/atlas/trading/signal", "/robot/trading/signal"),
            ],
        ),

        # Nodo de monitoreo de batería del robot
        Node(
            package="atlas_code_quant",
            executable="atlas_battery_monitor",
            name="atlas_battery_monitor",
            output="screen",
            parameters=[{"publish_rate_hz": 1.0}],
        ),
    ])
