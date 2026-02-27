"""
Tronco Encefalico Atlas: Funciones vitales y seguridad.

Analogo biologico: Bulbo raquideo + formacion reticular
- VitalsMonitor: Monitoreo de signos vitales
- SafetyPolicy: Politicas de seguridad
- GlobalState: Estados globales del sistema
- Watchdog: Vigilancia y heartbeats
"""
from .vitals_monitor import VitalsMonitor, VitalsReading
from .safety_policy import SafetyPolicy, SafetyVerdict, SafetyRule
from .global_state import GlobalState, SystemMode
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
