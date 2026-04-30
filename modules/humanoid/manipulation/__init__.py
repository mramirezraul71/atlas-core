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
from .arm_kinematics import ArmConfig, ArmKinematics
from .grasp_executor import GraspExecutor, GraspResult
from .grasp_planner import GraspCandidate, GraspConfig, GraspPlanner
from .hand_controller import FingerState, HandController
from .object_pose import ObjectPose, ObjectPoseEstimator

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
