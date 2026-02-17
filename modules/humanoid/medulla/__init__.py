"""
Médula Atlas: Bus de baja latencia para comunicación sensoriomotora.

Análogo biológico: Médula espinal
- Conecta sensores y actuadores con el SNC
- Transmisión de señales de alta velocidad (<1ms)
- Reflejos básicos sin intervención cortical
"""
from .bus import MedullaAtlas, get_medulla
from .schemas import SensorReading, MotorCommand, WorldState, ActionDecision

__all__ = [
    "MedullaAtlas",
    "get_medulla",
    "SensorReading",
    "MotorCommand",
    "WorldState",
    "ActionDecision",
]
