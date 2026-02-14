"""Orquestador: cerebro central. Bucle infinito Snapshot -> delegación a sub-agentes. Nunca se detiene."""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any, Dict

logger = logging.getLogger("atlas.orchestrator")

ORCHESTRATOR_TICK = float(__import__("os").environ.get("ORCHESTRATOR_TICK_SEC", "5.0"))
BASE = Path(__file__).resolve().parent.parent


def _snapshot() -> Dict[str, Any]:
    """Estado general del sistema para delegar tareas."""
    try:
        snapshot = {
            "ts": time.time(),
            "tick": ORCHESTRATOR_TICK,
            "base": str(BASE),
            "inspector_pending": True,
            "coder_pending": False,
            "navigator_pending": False,
        }
        return snapshot
    except Exception as e:
        logger.exception("snapshot error: %s", e)
        return {"ts": time.time(), "error": str(e)}


def _run_inspector(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """Delega al Inspector; nunca propaga excepción."""
    try:
        from .inspector_agent import inspect_system
        return inspect_system(snapshot)
    except Exception as e:
        logger.error("inspector_agent failed: %s", e)
        return {"ok": False, "error": str(e), "diagnosis": None}


def _run_coder(snapshot: Dict[str, Any], order: str) -> Dict[str, Any]:
    """Delega al Codificador; nunca propaga excepción."""
    try:
        from .coder_agent import execute_order
        return execute_order(order)
    except Exception as e:
        logger.error("coder_agent failed: %s", e)
        return {"ok": False, "error": str(e), "output": None}


def _run_navigator(snapshot: Dict[str, Any], url: str) -> Dict[str, Any]:
    """Delega al Navegador; nunca propaga excepción."""
    try:
        from .navigator_agent import fetch_page_text
        import asyncio
        text = asyncio.run(fetch_page_text(url))
        return {"ok": True, "url": url, "text_preview": (text or "")[:500]}
    except Exception as e:
        logger.error("navigator_agent failed: %s", e)
        return {"ok": False, "error": str(e), "url": url}


def run_orchestrator_cycle() -> None:
    """Bucle infinito: Snapshot -> delegar -> registrar -> repetir. Try/except global."""
    logger.info("Orquestador iniciado (tick=%.1fs)", ORCHESTRATOR_TICK)
    cycle = 0
    while True:
        cycle += 1
        try:
            snapshot = _snapshot()
            if snapshot.get("error"):
                logger.warning("snapshot con error: %s", snapshot["error"])
                time.sleep(ORCHESTRATOR_TICK)
                continue

            inspector_result = _run_inspector(snapshot)
            if inspector_result.get("diagnosis"):
                logger.info("[Inspector] diagnóstico: %s", inspector_result.get("diagnosis", "")[:200])
            if not inspector_result.get("ok") and inspector_result.get("error"):
                logger.debug("[Inspector] fallo: %s", inspector_result["error"])

            if cycle % 3 == 0:
                coder_result = _run_coder(snapshot, "Genera un script que imprima 'Hello from CoderAgent' y la fecha.")
                if coder_result.get("ok"):
                    logger.info("[Coder] ejecutado: %s", coder_result.get("output", "")[:150])
                else:
                    logger.debug("[Coder] fallo: %s", coder_result.get("error", ""))

            if cycle % 5 == 0:
                nav_result = _run_navigator(snapshot, "https://example.com")
                if nav_result.get("ok"):
                    logger.info("[Navigator] %s -> %s chars", nav_result.get("url"), len(nav_result.get("text_preview", "")))
                else:
                    logger.debug("[Navigator] fallo: %s", nav_result.get("error", ""))

        except KeyboardInterrupt:
            logger.info("Orquestador detenido por usuario")
            raise
        except Exception as e:
            logger.exception("orchestrator cycle error (continúa): %s", e)
        time.sleep(ORCHESTRATOR_TICK)
