"""
Sistema de Aprendizaje Atlas.

Interfaces para aprendizaje y mejora continua:
- DemonstrationLearning: Aprendizaje por demostracion (LfD)
- ReinforcementLearning: Aprendizaje por refuerzo (RL)
- NaturalLanguageFeedback: Feedback en lenguaje natural
- LearningAPI: API unificada para todas las interfaces
"""
from .api import LearningAPI
from .demonstration import (BehavioralCloning, Demonstration,
                            DemonstrationLearning)
from .natural_feedback import (FeedbackIntent, FeedbackType,
                               NaturalLanguageFeedback)
from .reinforcement import (Experience, Policy, ReinforcementLearning,
                            ReplayBuffer)

__all__ = [
    # Demonstration Learning
    "DemonstrationLearning",
    "Demonstration",
    "BehavioralCloning",
    # Reinforcement Learning
    "ReinforcementLearning",
    "Policy",
    "Experience",
    "ReplayBuffer",
    # Natural Language Feedback
    "NaturalLanguageFeedback",
    "FeedbackType",
    "FeedbackIntent",
    # Unified API
    "LearningAPI",
]
