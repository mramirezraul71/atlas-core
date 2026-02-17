"""
ATLAS Humanoid: Sistema cognitivo completo bio-inspirado.

Arquitectura Cognitiva:
- Medulla: Bus de comunicación de baja latencia (ZeroMQ)
- Cortex: Procesamiento sensorial y cognitivo
  - Frontal: Planificación y toma de decisiones
  - Parietal: Fusión sensorial y representación espacial
  - Temporal: Procesamiento auditivo y lenguaje
  - Occipital: Procesamiento visual
- Limbic: Sistema motivacional (goals, rewards, states)
- Hippo: Sistema de memoria (episódica, semántica)
- Brainstem: Monitoreo vital y seguridad
- Basal: Selección e inhibición de acciones
- Motor: Control de movimiento y trayectorias
- Learning: Interfaces de aprendizaje (LfD, RL, NL feedback)

Módulos Legacy:
- Kernel, Brain, Hands, Eyes, Ears, Autonomy, Comms, Update
"""
from __future__ import annotations

# Legacy modules
from modules.humanoid.kernel import Kernel
from modules.humanoid.brain import BrainOrchestrator
from modules.humanoid.hands import HandsModule
from modules.humanoid.eyes import EyesModule
from modules.humanoid.ears import EarsModule
from modules.humanoid.autonomy import AutonomyModule
from modules.humanoid.comms import CommsModule
from modules.humanoid.update import UpdateModule

# Cognitive Architecture - lazy imports to avoid circular dependencies
def get_medulla():
    """Get Medulla Atlas bus."""
    from modules.humanoid.medulla import MedullaAtlas
    return MedullaAtlas

def get_cortex():
    """Get Cortex modules."""
    from modules.humanoid import cortex
    return cortex

def get_limbic():
    """Get Limbic system modules."""
    from modules.humanoid import limbic
    return limbic

def get_hippo():
    """Get Hippocampus memory modules."""
    from modules.humanoid import hippo
    return hippo

def get_brainstem():
    """Get Brainstem modules."""
    from modules.humanoid import brainstem
    return brainstem

def get_basal():
    """Get Basal ganglia modules."""
    from modules.humanoid import basal
    return basal

def get_motor():
    """Get Motor control modules."""
    from modules.humanoid import motor
    return motor

def get_learning():
    """Get Learning modules."""
    from modules.humanoid import learning
    return learning


def create_humanoid_kernel() -> Kernel:
    """Build kernel and register all modules. Brain wired to Autonomy."""
    k = Kernel()
    brain = BrainOrchestrator()
    k.register(brain)
    k.register(HandsModule())
    k.register(EyesModule())
    k.register(EarsModule())
    autonomy = AutonomyModule()
    autonomy.set_brain(brain)
    k.register(autonomy)
    k.register(CommsModule())
    k.register(UpdateModule())
    return k


def create_cognitive_system():
    """
    Creates full cognitive system with all Atlas brain components.
    
    Returns:
        Dictionary with all cognitive modules initialized.
    """
    from modules.humanoid.medulla import MedullaAtlas
    from modules.humanoid.cortex.frontal import TaskPlanner, DecisionMaker, InhibitoryControl
    from modules.humanoid.cortex.parietal import SensoryFusion, SpatialMap, BodySchema
    from modules.humanoid.cortex.temporal import AudioProcessor, LanguageUnderstanding, EpisodicRecall
    from modules.humanoid.cortex.occipital import VisionPipeline, DepthEstimation, ObjectRecognition
    from modules.humanoid.limbic import GoalManager, RewardEngine, StateRegulator
    from modules.humanoid.hippo import HippoAPI
    from modules.humanoid.brainstem import VitalsMonitor, SafetyPolicy, GlobalState, Watchdog
    from modules.humanoid.basal import ActionSelector, Inhibitor
    from modules.humanoid.motor import TrajectoryPlanner, MotorController, MotorInterface
    from modules.humanoid.learning import LearningAPI
    
    # Initialize bus
    medulla = MedullaAtlas(use_zmq=False)  # Use threading bus by default
    
    # Initialize memory
    hippo = HippoAPI()
    
    # Initialize cortex
    cortex = {
        "frontal": {
            "task_planner": TaskPlanner(),
            "decision_maker": DecisionMaker(),
            "inhibitory_control": InhibitoryControl(),
        },
        "parietal": {
            "sensory_fusion": SensoryFusion(),
            "spatial_map": SpatialMap(),
            "body_schema": BodySchema(),
        },
        "temporal": {
            "audio_processor": AudioProcessor(),
            "language_understanding": LanguageUnderstanding(),
            "episodic_recall": EpisodicRecall(hippo_api=hippo),
        },
        "occipital": {
            "vision_pipeline": VisionPipeline(),
            "depth_estimation": DepthEstimation(),
            "object_recognition": ObjectRecognition(),
        },
    }
    
    # Initialize limbic
    limbic = {
        "goal_manager": GoalManager(),
        "reward_engine": RewardEngine(),
        "state_regulator": StateRegulator(),
    }
    
    # Initialize brainstem
    brainstem = {
        "vitals_monitor": VitalsMonitor(),
        "safety_policy": SafetyPolicy(),
        "global_state": GlobalState(),
        "watchdog": Watchdog(),
    }
    
    # Initialize basal ganglia
    basal = {
        "action_selector": ActionSelector(),
        "inhibitor": Inhibitor(),
    }
    
    # Initialize motor control
    trajectory_planner = TrajectoryPlanner()
    motor_controller = MotorController()
    motor_interface = MotorInterface(trajectory_planner, motor_controller)
    
    motor = {
        "trajectory_planner": trajectory_planner,
        "motor_controller": motor_controller,
        "motor_interface": motor_interface,
    }
    
    # Initialize learning
    learning = LearningAPI()
    
    return {
        "medulla": medulla,
        "cortex": cortex,
        "limbic": limbic,
        "hippo": hippo,
        "brainstem": brainstem,
        "basal": basal,
        "motor": motor,
        "learning": learning,
    }


_humanoid_kernel: Kernel | None = None
_cognitive_system: dict | None = None


def get_humanoid_kernel() -> Kernel:
    """Get or create the humanoid kernel."""
    global _humanoid_kernel
    if _humanoid_kernel is None:
        _humanoid_kernel = create_humanoid_kernel()
    return _humanoid_kernel


def get_cognitive_system() -> dict:
    """Get or create the cognitive system."""
    global _cognitive_system
    if _cognitive_system is None:
        _cognitive_system = create_cognitive_system()
    return _cognitive_system


__all__ = [
    # Legacy
    "Kernel",
    "BrainOrchestrator",
    "HandsModule",
    "EyesModule",
    "EarsModule",
    "AutonomyModule",
    "CommsModule",
    "UpdateModule",
    "create_humanoid_kernel",
    "get_humanoid_kernel",
    # Cognitive Architecture
    "create_cognitive_system",
    "get_cognitive_system",
    "get_medulla",
    "get_cortex",
    "get_limbic",
    "get_hippo",
    "get_brainstem",
    "get_basal",
    "get_motor",
    "get_learning",
]
