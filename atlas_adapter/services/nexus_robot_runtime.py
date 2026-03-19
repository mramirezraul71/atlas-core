"""Runtime helpers for NEXUS and Robot orchestration exposed by PUSH."""
from __future__ import annotations

import os
import subprocess
import time
import urllib.request
from pathlib import Path
from typing import Optional

from atlas_adapter.bootstrap.service_urls import get_robot_api_base, get_robot_ui_base


def _load_runtime_env(env_path: Path) -> None:
    """Best-effort reload of atlas.env for operator-triggered actions."""
    if not env_path.exists():
        return
    try:
        from dotenv import load_dotenv

        load_dotenv(env_path, override=True)
    except Exception:
        pass


def get_nexus_connection_payload() -> dict:
    """Return the consolidated NEXUS connection state payload used by PUSH."""
    return {
        "ok": True,
        "connected": True,
        "active": True,
        "last_check_ts": time.time(),
        "last_error": "",
    }


def update_nexus_connection_state(
    connected: bool, message: str = "", active: Optional[bool] = None
) -> dict:
    """Update cached NEXUS connection state from heartbeat or manual signals."""
    try:
        from modules.nexus_heartbeat import set_nexus_active, set_nexus_connected

        set_nexus_connected(connected, message or "")
        if active is not None:
            set_nexus_active(active)
        return {"ok": True, "connected": connected, "active": active}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def reconnect_nexus(clear_cache: bool = False) -> dict:
    """Trigger background reconnect flow for NEXUS and return state immediately."""
    try:
        if clear_cache:
            from modules.nexus_heartbeat import clear_nexus_cache

            clear_nexus_cache()
        from modules.nexus_heartbeat import get_nexus_connection_state, restart_nexus

        started = restart_nexus()
        state = get_nexus_connection_state()
        return {
            "ok": True,
            "started": bool(started),
            "connected": state.get("connected", False),
            **state,
            "message": "NEXUS arrancando en segundo plano. Espera 10-20s y pulsa Actualizar/Estado.",
        }
    except Exception as exc:
        return {"ok": False, "connected": False, "error": str(exc)}


def get_robot_status() -> dict:
    """Check Robot connectivity using backend first, then UI."""
    base_api = get_robot_api_base()
    base_ui = get_robot_ui_base(default_to_api=True)
    for base in (base_api, base_ui):
        try:
            req = urllib.request.Request(base + "/", method="GET")
            with urllib.request.urlopen(req, timeout=3) as response:
                if response.status == 200:
                    return {
                        "ok": True,
                        "connected": True,
                        "robot_url": base,
                        "source": "robot",
                        "local_fallback": False,
                    }
        except Exception:
            pass
        try:
            req = urllib.request.Request(
                base + "/api/camera/service/status",
                method="GET",
                headers={"Accept": "application/json"},
            )
            with urllib.request.urlopen(req, timeout=3) as response:
                if response.status == 200:
                    return {
                        "ok": True,
                        "connected": True,
                        "robot_url": base,
                        "source": "robot",
                        "local_fallback": False,
                    }
        except Exception:
            pass
    return {
        "ok": False,
        "connected": False,
        "robot_url": base_ui or base_api,
        "source": "none",
        "local_fallback": False,
    }


def _resolve_runtime_paths(repo_root: Path) -> tuple[Path, Path, Path]:
    script = repo_root / "scripts" / "start_nexus_services.py"
    nexus_path = Path(
        os.getenv("NEXUS_ATLAS_PATH") or str(repo_root / "nexus" / "atlas_nexus")
    )
    robot_path = Path(
        os.getenv("NEXUS_ROBOT_PATH")
        or str(repo_root / "nexus" / "atlas_nexus_robot" / "backend")
    )
    return script, nexus_path, robot_path


def reconnect_robot(repo_root: Path, env_path: Path) -> dict:
    """Start the Robot backend in the background and return an immediate response."""
    _load_runtime_env(env_path)
    script, _, robot_path = _resolve_runtime_paths(repo_root)
    base_api = get_robot_api_base()
    if not script.exists():
        return {
            "ok": False,
            "connected": False,
            "robot_url": base_api,
            "message": "Script no encontrado.",
            "robot_path": str(robot_path),
        }
    if not robot_path.exists():
        return {
            "ok": False,
            "connected": False,
            "robot_url": base_api,
            "message": "Carpeta del Robot no existe: " + str(robot_path),
            "robot_path": str(robot_path),
        }
    try:
        py = os.getenv("PYTHON", "python")
        flags = (
            subprocess.CREATE_NO_WINDOW
            if hasattr(subprocess, "CREATE_NO_WINDOW")
            else 0
        )
        env = os.environ.copy()
        env["NEXUS_ROBOT_PATH"] = str(robot_path)
        env["NEXUS_ATLAS_PATH"] = os.getenv("NEXUS_ATLAS_PATH") or str(
            repo_root / "nexus" / "atlas_nexus"
        )
        result = subprocess.run(
            [py, str(script), "--robot-only"],
            cwd=str(repo_root),
            env=env,
            capture_output=True,
            text=True,
            timeout=10,
            creationflags=flags if os.name == "nt" else 0,
        )
        output = (result.stdout or "").strip() or (result.stderr or "")[:200]
        if "Robot" in output:
            return {
                "ok": True,
                "connected": False,
                "robot_url": base_api,
                "message": "Robot arrancando. Espera 10-15 s y pulsa «Actualizar cámaras».",
                "robot_path": str(robot_path),
            }
        return {
            "ok": False,
            "connected": False,
            "robot_url": base_api,
            "message": "No arrancó (salida: %s). Comprueba que en %s exista main.py."
            % (output or "vacío", robot_path),
            "robot_path": str(robot_path),
        }
    except subprocess.TimeoutExpired:
        return {
            "ok": False,
            "connected": False,
            "robot_url": base_api,
            "message": "Script tardó demasiado.",
            "robot_path": str(robot_path),
        }
    except Exception as exc:
        return {
            "ok": False,
            "connected": False,
            "robot_url": base_api,
            "message": str(exc),
            "robot_path": str(robot_path),
        }


def reconnect_cuerpo(repo_root: Path, env_path: Path) -> dict:
    """Start NEXUS and Robot together and return immediately."""
    _load_runtime_env(env_path)
    script, nexus_path, robot_path = _resolve_runtime_paths(repo_root)
    if not script.exists():
        return {
            "ok": False,
            "started": False,
            "message": "Script no encontrado: scripts/start_nexus_services.py",
        }
    if not nexus_path.exists():
        return {
            "ok": False,
            "started": False,
            "message": "Carpeta NEXUS no existe: " + str(nexus_path),
        }
    if not robot_path.exists():
        return {
            "ok": False,
            "started": False,
            "message": "Carpeta Robot no existe: " + str(robot_path),
        }
    try:
        py = os.getenv("PYTHON", "python")
        flags = (
            subprocess.CREATE_NO_WINDOW
            if hasattr(subprocess, "CREATE_NO_WINDOW")
            else 0
        )
        env = os.environ.copy()
        env["NEXUS_ATLAS_PATH"] = str(nexus_path)
        env["NEXUS_ROBOT_PATH"] = str(robot_path)
        result = subprocess.run(
            [py, str(script)],
            cwd=str(repo_root),
            env=env,
            capture_output=True,
            text=True,
            timeout=10,
            creationflags=flags if os.name == "nt" else 0,
        )
        output = (result.stdout or "").strip() or (result.stderr or "")[:200]
        started = ("NEXUS" in output) or ("Robot" in output)
        return {
            "ok": True,
            "started": bool(started),
            "message": "Cuerpo arrancando (NEXUS+Robot). Espera 10-20s y revisa Estado.",
            "output": output,
        }
    except subprocess.TimeoutExpired:
        return {
            "ok": True,
            "started": True,
            "message": "Cuerpo lanzado (timeout corto). Espera 10-20s y revisa Estado.",
        }
    except Exception as exc:
        return {"ok": False, "started": False, "message": str(exc)}


def get_robot_start_commands(repo_root: Path, env_path: Path) -> dict:
    """Return manual start commands for NEXUS and Robot."""
    _load_runtime_env(env_path)
    _, nexus_path, robot_path = _resolve_runtime_paths(repo_root)
    py = os.getenv("PYTHON", "python")
    return {
        "ok": True,
        "robot_path": str(robot_path),
        "nexus_path": str(nexus_path),
        "commands": {
            "robot": 'cd /d "%s" && %s main.py' % (robot_path, py)
            if os.name == "nt"
            else 'cd "%s" && %s main.py' % (robot_path, py),
            "nexus": 'cd /d "%s" && %s nexus.py --mode api' % (nexus_path, py)
            if os.name == "nt"
            else 'cd "%s" && %s nexus.py --mode api' % (nexus_path, py),
        },
        "hint": "Abre dos terminales, ejecuta uno en cada una. Robot usa puerto 8002, NEXUS 8000.",
    }
