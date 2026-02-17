"""
Sistema de Aprendizaje Atlas.

Interfaces para aprendizaje y mejora continua:
- DemonstrationLearning: Aprendizaje por demostracion (LfD)
- ReinforcementLearning: Aprendizaje por refuerzo (RL)
- NaturalLanguageFeedback: Feedback en lenguaje natural
- LearningAPI: API unificada para todas las interfaces
"""
from .demonstration import DemonstrationLearning, Demonstration, BehavioralCloning
from .reinforcement import ReinforcementLearning, Policy, Experience, ReplayBuffer
from .natural_feedback import NaturalLanguageFeedback, FeedbackType, FeedbackIntent
from .api import LearningAPI

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
