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
from .slam import SLAMEngine, SLAMConfig
from .localization import Localizer, PoseEstimate
from .planner import PathPlanner, Path, Waypoint
from .controller import PathController, ControlCommand
from .costmap import Costmap2D, CostmapConfig
from .recovery import RecoveryBehavior, RecoveryManager
from .navigation_system import NavigationSystem, NavigationConfig, NavigationState

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
