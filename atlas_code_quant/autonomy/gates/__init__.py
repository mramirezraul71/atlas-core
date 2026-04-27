"""autonomy.gates — gates del orquestador autónomo.

* F1 scaffold: ``contracts.py`` (GateInput/GateOutput/GateDecision).
* F17: ``orchestrator_gates.py`` con la interfaz ``Gate`` y stubs
  (HealthGate, RadarGate, StrategyGate, RiskGate, VisionGate,
  BrokerGate, LiveGate, KillSwitchGate).

F18+ irá conectando los gates a su lógica real (Vision, risk, kill
switch, etc.) sin romper la interfaz F17.
"""

from atlas_code_quant.autonomy.gates.orchestrator_gates import (
    BrokerGate,
    Gate,
    GateResult,
    HealthGate,
    KillSwitchGate,
    LiveGate,
    RadarGate,
    RiskGate,
    StrategyGate,
    VisionGate,
)

__all__ = [
    "Gate",
    "GateResult",
    "HealthGate",
    "RadarGate",
    "StrategyGate",
    "RiskGate",
    "VisionGate",
    "BrokerGate",
    "LiveGate",
    "KillSwitchGate",
]
