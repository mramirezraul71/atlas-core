"""
ATLAS Simulation Module
========================
Sistema de simulación y digital twin para entrenamiento.

Componentes:
- MuJoCo Environment: Entornos de simulación física
- Robot Model: Modelos URDF/MJCF del robot
- Domain Randomization: Variación de parámetros
- Sim2Real: Transferencia simulación-realidad
- Training: Pipelines de entrenamiento RL

Uso:
    from simulation import SimulationEngine

    sim = SimulationEngine()
    sim.load_robot("atlas_humanoid")
    sim.run_episode(policy)
"""
from .domain_randomization import DomainRandomizer
from .robot_model import ModelConfig, RobotModel
from .sim2real import Sim2RealTransfer
from .simulation_engine import SimConfig, SimulationEngine
from .training_env import TrainingEnvironment

__all__ = [
    "SimulationEngine",
    "SimConfig",
    "RobotModel",
    "ModelConfig",
    "DomainRandomizer",
    "Sim2RealTransfer",
    "TrainingEnvironment",
]
