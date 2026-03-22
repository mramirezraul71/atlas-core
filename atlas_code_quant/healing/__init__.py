"""Atlas Code-Quant — Self-Healing & Error Memory.

Módulo 6: LLM + ChromaDB memoria episódica, auto-repair <30s.
"""
from .error_agent import ErrorMemoryAgent, ErrorRecord, RepairAction
from .self_healing import SelfHealingOrchestrator, HealingStatus

__all__ = [
    "ErrorMemoryAgent",
    "ErrorRecord",
    "RepairAction",
    "SelfHealingOrchestrator",
    "HealingStatus",
]
