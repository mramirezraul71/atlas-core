"""
Médula Atlas: Bus de baja latencia para comunicación sensoriomotora.

Análogo biológico: Médula espinal
- Conecta sensores y actuadores con el SNC
- Transmisión de señales de alta velocidad (<1ms)
- Reflejos básicos sin intervención cortical
"""
from .bus import MedullaAtlas, SharedState, get_medulla
from .schemas import SensorReading, MotorCommand, WorldState, ActionDecision

__all__ = [
    "MedullaAtlas",
    "SharedState",
    "get_medulla",
    "SensorReading",
    "MotorCommand",
    "WorldState",
    "ActionDecision",
]
