"""
Tronco Encefalico Atlas: Funciones vitales y seguridad.

Analogo biologico: Bulbo raquideo + formacion reticular
- VitalsMonitor: Monitoreo de signos vitales
- SafetyPolicy: Politicas de seguridad
- GlobalState: Estados globales del sistema
- Watchdog: Vigilancia y heartbeats
"""
from .global_state import GlobalState, SystemMode
from .safety_policy import SafetyPolicy, SafetyRule, SafetyVerdict
from .vitals_monitor import VitalsMonitor, VitalsReading
from .watchdog import Watchdog

__all__ = [
    "VitalsMonitor",
    "VitalsReading",
    "SafetyPolicy",
    "SafetyVerdict",
    "SafetyRule",
    "GlobalState",
    "SystemMode",
    "Watchdog",
]
