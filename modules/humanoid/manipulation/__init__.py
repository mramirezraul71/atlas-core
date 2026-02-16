"""
ATLAS Manipulation Module
==========================
Sistema de manipulación y grasping para brazos y manos.

Componentes:
- GraspPlanner: Planificación de agarres
- GraspExecutor: Ejecución de agarres
- HandController: Control de manos dextrosas
- ArmKinematics: Cinemática de brazos
- ObjectPose: Estimación de pose de objetos

Uso:
    from modules.humanoid.manipulation import GraspPlanner, GraspExecutor
    
    planner = GraspPlanner()
    grasps = planner.plan_grasp(object_point_cloud)
    executor = GraspExecutor()
    executor.execute(grasps[0])
"""
from .grasp_planner import GraspPlanner, GraspCandidate, GraspConfig
from .grasp_executor import GraspExecutor, GraspResult
from .hand_controller import HandController, FingerState
from .arm_kinematics import ArmKinematics, ArmConfig
from .object_pose import ObjectPoseEstimator, ObjectPose

__all__ = [
    "GraspPlanner",
    "GraspCandidate",
    "GraspConfig",
    "GraspExecutor",
    "GraspResult",
    "HandController",
    "FingerState",
    "ArmKinematics",
    "ArmConfig",
    "ObjectPoseEstimator",
    "ObjectPose",
]
