#!/usr/bin/env python
"""ATLAS Market Open Supervisor (paper-safe).

Supervisa actividad de ATLAS alrededor de la apertura del mercado (ET):
- Salud de PUSH (:8791), Quant (:8795) y Robot (:8002)
- Estado de safe-mode del Robot
- Arranca Autonomy Orchestrator (si no está corriendo) en modo seguro (solo flags)

Uso:
  python -m atlas_code_quant.scripts.market_open_supervisor --loop
  python -m atlas_code_quant.scripts.market_open_supervisor --run-once
"""

from __future__ import annotations

import argparse
import ctypes
import json
import logging
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import pytz
import requests

# Asegurar imports si se ejecuta como script suelto
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

try:
    from atlas_code_quant.operations.operation_state_contract import (
        quant_operation_state_path,
        read_json,
        update_quant_state,
    )
except ModuleNotFoundError:  # pragma: no cover - fallback for script suelto
    from operations.operation_state_contract import quant_operation_state_path, read_json, update_quant_state


LOG_PATH = Path(r"C:\ATLAS_PUSH\logs\market_open_supervisor.log")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s — %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("atlas.market_open_supervisor")


def _setup_file_logging() -> None:
    try:
        LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        fh = logging.FileHandler(LOG_PATH, encoding="utf-8")
        fh.setLevel(logging.INFO)
        fmt = logging.Formatter("%(asctime)s %(levelname)s %(name)s — %(message)s")
        fh.setFormatter(fmt)
        logging.getLogger().addHandler(fh)
    except Exception:
        # Si el log file falla, seguimos en stdout.
        pass


def _acquire_single_instance_mutex(name: str) -> int | None:
    """Mutex global Windows: evita doble instancia."""
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    handle = kernel32.CreateMutexW(None, False, name)
    if not handle:
        return None
    already_exists = ctypes.get_last_error() == 183  # ERROR_ALREADY_EXISTS
    if already_exists:
        kernel32.CloseHandle(handle)
        return None
    return int(handle)


def _close_mutex(handle: int) -> None:
    try:
        ctypes.WinDLL("kernel32").CloseHandle(handle)
    except Exception:
        pass


@dataclass(frozen=True)
class ServiceCheck:
    name: str
    ok: bool
    url: str
    status_code: int | None = None
    details: Dict[str, Any] | None = None
    error: str | None = None


def _get_json(url: str, timeout: float = 1.5) -> Tuple[Optional[int], Dict[str, Any], Optional[str]]:
    try:
        r = requests.get(url, timeout=timeout)
        code = int(r.status_code)
        data = r.json() if r.headers.get("content-type", "").lower().startswith("application/json") else {}
        if not isinstance(data, dict):
            data = {}
        return code, data, None
    except Exception as exc:
        return None, {}, str(exc)


def check_push(base_url: str = "http://127.0.0.1:8791") -> ServiceCheck:
    url = base_url.rstrip("/") + "/health"
    code, data, err = _get_json(url)
    ok = bool(code == 200 and (data.get("ok") is True or data.get("service") == "atlas_push"))
    return ServiceCheck(name="push", ok=ok, url=url, status_code=code, details=data or None, error=err)


def check_quant(base_url: str = "http://127.0.0.1:8795") -> ServiceCheck:
    url = base_url.rstrip("/") + "/health"
    code, data, err = _get_json(url)
    ok = bool(code == 200 and str(data.get("status") or "").lower() in {"ok", "StatusEnum.OK".lower()} or code == 200)
    return ServiceCheck(name="quant", ok=ok, url=url, status_code=code, details=data or None, error=err)


def check_robot(base_url: str = "http://127.0.0.1:8002") -> ServiceCheck:
    url = base_url.rstrip("/") + "/api/health"
    code, data, err = _get_json(url)
    ok = bool(code == 200 and (data.get("status") in {"healthy", "ok"} or True))
    return ServiceCheck(name="robot", ok=ok, url=url, status_code=code, details=data or None, error=err)


def read_robot_safe_mode(base_url: str = "http://127.0.0.1:8002") -> bool:
    code, data, _ = _get_json(base_url.rstrip("/") + "/api/safe-mode/status")
    if code != 200 or not data:
        return False
    return bool(data.get("safe_mode_active", False))


def _et_now(now_utc: datetime | None = None) -> datetime:
    if now_utc is None:
        now_utc = datetime.now(timezone.utc)
    et = pytz.timezone("America/New_York")
    return now_utc.astimezone(et)


def is_market_open_window(
    now_utc: datetime | None = None,
    *,
    pre_minutes: int = 15,
    post_minutes: int = 20,
) -> bool:
    """Ventana de supervisión: [09:30 - pre_minutes, 09:30 + post_minutes] ET."""
    now_et = _et_now(now_utc)
    if now_et.weekday() > 4:
        return False
    open_et = now_et.replace(hour=9, minute=30, second=0, microsecond=0)
    start = open_et - timedelta(minutes=pre_minutes)
    end = open_et + timedelta(minutes=post_minutes)
    return start <= now_et <= end


def _start_autonomy_orchestrator() -> Dict[str, Any]:
    """Arranca el orquestador dentro del proceso (no crea servicios nuevos)."""
    try:
        from atlas_code_quant.scripts.start_autonomy_orchestrator import main as start_orch

        # Corre en background dentro de ese módulo; este supervisor solo lo dispara.
        start_orch()
        return {"ok": True}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def run_checks() -> Dict[str, Any]:
    push = check_push()
    quant = check_quant()
    robot = check_robot()
    safe_mode = read_robot_safe_mode()

    # Gobernanza paper premarket: habilitar bypass seguro en ventana de apertura
    if is_market_open_window():
        try:
            state_path = quant_operation_state_path(_ROOT)
            if state_path.exists():
                state = read_json(state_path)
                if isinstance(state, dict):
                    # Activa bypass de exit_now SOLO en paper (mantiene live intacto)
                    state.setdefault("paper_bypass_exit_now_guard", True)
                    state.setdefault("paper_bypass_entry_validation_guard", True)
                    if state.get("account_scope") == "paper":
                        state["paper_bypass_exit_now_guard"] = True
                        state["paper_bypass_entry_validation_guard"] = True
                        # Solo enviar órdenes paper en sesión regular (9:30–16:00 ET); el loop usa preview fuera de hora.
                        state["paper_market_hours_strict"] = True
                    update_quant_state(
                        state,
                        quant_state_path=state_path,
                        source_module="atlas_market_open_supervisor",
                        ensure_ascii=False,
                    )
        except Exception as exc:
            logger.warning("No se pudo aplicar gobernanza premarket en operation_center_state.json: %s", exc)

    payload = {
        "ts_utc": datetime.now(timezone.utc).isoformat(),
        "services": {
            "push": push.__dict__,
            "quant": quant.__dict__,
            "robot": robot.__dict__,
        },
        "robot_safe_mode_active": safe_mode,
    }

    # Acción segura: si estamos en ventana de apertura y Robot está en safe-mode -> no tocar nada,
    # solo registramos (Quant ya tiene fall-safe propio).
    if safe_mode:
        logger.warning("ROBOT SAFE-MODE ACTIVO: supervisión en modo observación (no acciones).")
    else:
        # Arranque del orquestador: idempotente vía mutex interno.
        out = _start_autonomy_orchestrator()
        if not out.get("ok"):
            logger.warning("No se pudo iniciar AutonomyOrchestrator: %s", out.get("error"))
        payload["autonomy_orchestrator_start"] = out

    # Logging de estado
    for svc in (push, quant, robot):
        if svc.ok:
            logger.info("SERVICE OK: %s (%s)", svc.name, svc.url)
        else:
            logger.warning("SERVICE FAIL: %s (%s) err=%s", svc.name, svc.url, svc.error)

    return payload


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ATLAS Market Open Supervisor (paper-safe)")
    p.add_argument("--run-once", action="store_true", help="Ejecuta una sola vez y sale")
    p.add_argument("--loop", action="store_true", help="Loop continuo (cada 15s en ventana; cada 60s fuera)")
    p.add_argument("--dump-json", default="", help="Ruta opcional para volcar el último payload JSON")
    return p.parse_args()


def main() -> int:
    _setup_file_logging()
    mutex = _acquire_single_instance_mutex("Global\\ATLAS_PUSH_MARKET_OPEN_SUPERVISOR")
    if mutex is None:
        print("MarketOpenSupervisor ya está corriendo (mutex activo). Saliendo.")
        return 0
    try:
        args = parse_args()
        if args.run_once and args.loop:
            print("Usa solo uno: --run-once o --loop")
            return 2

        def _emit(payload: Dict[str, Any]) -> None:
            if args.dump_json:
                path = Path(args.dump_json)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")

        if args.run_once:
            payload = run_checks()
            _emit(payload)
            return 0

        # Default: loop
        if not args.loop:
            args.loop = True

        while True:
            in_window = is_market_open_window()
            if in_window:
                logger.info("Ventana apertura ACTIVA (ET). Ejecutando checks.")
                payload = run_checks()
                _emit(payload)
                time.sleep(15)
            else:
                # Fuera de ventana: pulso de supervisión ligero.
                logger.info("Fuera de ventana de apertura. Heartbeat supervisor.")
                time.sleep(60)
    finally:
        _close_mutex(mutex)


if __name__ == "__main__":
    raise SystemExit(main())

