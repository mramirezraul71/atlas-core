"""
Sistema Limbico Atlas: Gestion de objetivos, recompensas y estado interno.

Analogo biologico: Sistema limbico (amigdala, hipotalamo, nucleo accumbens)
- GoalManager: Gestion de objetivos y prioridades
- RewardEngine: Funcion de recompensa y senales de refuerzo
- StateRegulator: Regulacion del estado interno
"""
from .goal_manager import GoalManager, Goal
from .reward_engine import RewardEngine, RewardSignal
from .state_regulator import StateRegulator, InternalState

__all__ = [
    "GoalManager",
    "Goal",
    "RewardEngine",
    "RewardSignal",
    "StateRegulator",
    "InternalState",
]
