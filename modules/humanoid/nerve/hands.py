"""Nervio: manos. Ejecución local (mouse/teclado vía hands_eyes). El nervio llega al mouse de esta máquina."""
from __future__ import annotations

from typing import Any, Dict, Optional


def hands_execute(
    action: str,
    payload: Dict[str, Any],
    verify_before: bool = False,
    verify_after: bool = False,
) -> Dict[str, Any]:
    """
    Ejecuta acción de manos: click, type, hotkey, scroll.
    Hoy: siempre local (hands_eyes → pyautogui). El nervio llega al mouse de esta máquina.
    Retorna {ok, error?, evidence_before?, evidence_after?}.
    """
    try:
        from modules.humanoid.hands_eyes import execute_action
        return execute_action(action, payload, verify_before=verify_before, verify_after=verify_after)
    except Exception as e:
        return {"ok": False, "error": str(e), "evidence_before": None, "evidence_after": None}


def hands_locate(query: str, region: Optional[tuple] = None) -> Dict[str, Any]:
    """Localiza elemento en pantalla (ojos + coordenadas). Para usar antes de hands_execute(click)."""
    try:
        from modules.humanoid.hands_eyes import locate_element
        return locate_element(query, region=region)
    except Exception as e:
        return {"ok": False, "matches": [], "error": str(e)}
