"""
ATLAS Navigation Module
========================
Sistema de navegación autónoma con SLAM, localización y planificación.

Componentes:
- SLAM: Simultaneous Localization and Mapping
- Localization: AMCL, EKF, particle filter
- Planner: Path planning (A*, Dijkstra, RRT)
- Controller: Path following (DWA, TEB, Pure Pursuit)
- Costmap: Occupancy grid management
- Recovery: Recovery behaviors

Uso:
    from modules.humanoid.navigation import NavigationSystem

    nav = NavigationSystem()
    nav.start_mapping()
    nav.goto(x=1.0, y=2.0, theta=0.0)
"""
from .controller import ControlCommand, PathController
from .costmap import Costmap2D, CostmapConfig
from .localization import Localizer, PoseEstimate
from .navigation_system import (NavigationConfig, NavigationState,
                                NavigationSystem)
from .planner import Path, PathPlanner, Waypoint
from .recovery import RecoveryBehavior, RecoveryManager
from .slam import SLAMConfig, SLAMEngine

__all__ = [
    "SLAMEngine",
    "SLAMConfig",
    "Localizer",
    "PoseEstimate",
    "PathPlanner",
    "Path",
    "Waypoint",
    "PathController",
    "ControlCommand",
    "Costmap2D",
    "CostmapConfig",
    "RecoveryBehavior",
    "RecoveryManager",
    "NavigationSystem",
    "NavigationConfig",
    "NavigationState",
]
