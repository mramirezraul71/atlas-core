#!/usr/bin/env python3
"""ATLAS_EVOLUTION: Entorno de producción final. Integración credenciales + Visión + Daemon 12h.

  1. Integración de Credenciales: módulo en modules/evolution/credentials.py lee C:\\dev\\credenciales.txt (PyPI, GitHub, Hugging Face).
  2. Módulo de Visión: pytesseract -> C:\\Program Files\\Tesseract-OCR\\tesseract.exe; prueba snapshot + OCR en modules/evolution/vision.py.
  3. Daemon: Bucle asyncio cada 12h — PyPI (.temp_venv), GitHub (bots trading, app Rauli), Hugging Face (visión Insta360 Link 2).
  4. Autonomía Gobernada: Sandbox exitoso -> actualiza requirements.txt y reporta 'Asimilación Exitosa' en Dashboard ATLAS.

  Ejecución: python atlas_evolution.py  (o pythonw para segundo plano)
"""
from __future__ import annotations

import asyncio
import logging
import os
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))

# Config de producción antes de importar daemon
_env_file = BASE / "config" / "atlas.env"
if _env_file.exists():
    try:
        from dotenv import load_dotenv
        load_dotenv(_env_file, override=True)
    except ImportError:
        pass

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("atlas.evolution")

# Módulo de Visión: pytesseract + prueba snapshot/OCR
try:
    from modules.evolution.vision import configure_tesseract, test_vision_snapshot
    configure_tesseract(os.environ.get("TESSERACT_CMD") or r"C:\Program Files\Tesseract-OCR\tesseract.exe")
    _vision_ok, _vision_msg = test_vision_snapshot()
    logger.info("Visión: %s", _vision_msg)
except Exception as e:
    logger.warning("Visión (no bloqueante): %s", e)

# Sandbox unificado: cualquier hallazgo se prueba aquí
TEMP_GROWTH = BASE / "temp_growth"
TEMP_GROWTH_VENV = TEMP_GROWTH / "venv"
TEMP_GROWTH_WORKSPACE = TEMP_GROWTH / "workspace"
TEMP_GROWTH_MODELS = TEMP_GROWTH / "models"

# Ciclo cada 12 horas
SCAN_INTERVAL_12H = 43200


def _validate_pypi(creds: dict) -> tuple[bool, str]:
    """Valida conexión con la API de PyPI (público o con token)."""
    try:
        import urllib.request
        import base64
        from modules.evolution.credentials import get_pypi_auth, ensure_pypi_password
        pypi_user, pypi_pass = get_pypi_auth(creds)
        req = urllib.request.Request("https://pypi.org/pypi/requests/json", headers={"Accept": "application/json"})
        if pypi_user and pypi_pass:
            auth = base64.b64encode(f"{pypi_user}:{pypi_pass}".encode()).decode()
            req.add_header("Authorization", f"Basic {auth}")
        with urllib.request.urlopen(req, timeout=8) as r:
            data = __import__("json").loads(r.read().decode("utf-8"))
        ver = data.get("info", {}).get("version", "?")
        return True, f"PyPI OK (requests {ver})"
    except Exception as e:
        return False, f"PyPI: {e}"


def _validate_github(creds: dict) -> tuple[bool, str]:
    """Valida conexión con la API de GitHub usando token."""
    token = creds.get("github_token", "").strip()
    if not token:
        return False, "GitHub: falta github_token en credenciales"
    try:
        import urllib.request
        import urllib.error
        req = urllib.request.Request(
            "https://api.github.com/user",
            headers={"Authorization": f"Bearer {token}", "Accept": "application/vnd.github.v3+json"},
        )
        with urllib.request.urlopen(req, timeout=10) as r:
            data = __import__("json").loads(r.read().decode("utf-8"))
        login = data.get("login", "?")
        return True, f"GitHub OK (usuario {login})"
    except urllib.error.HTTPError as e:
        return False, f"GitHub: HTTP {e.code}"
    except Exception as e:
        return False, f"GitHub: {e}"


def _validate_huggingface(creds: dict) -> tuple[bool, str]:
    """Valida conexión con la API de Hugging Face usando token."""
    token = creds.get("hf_token", "").strip()
    if not token:
        return False, "Hugging Face: falta hf_token en credenciales"
    try:
        import urllib.request
        req = urllib.request.Request(
            "https://huggingface.co/api/models?limit=1",
            headers={"Authorization": f"Bearer {token}", "Accept": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=10) as r:
            __import__("json").loads(r.read().decode("utf-8"))
        return True, "Hugging Face OK"
    except Exception as e:
        return False, f"Hugging Face: {e}"


def fase_inicio() -> dict:
    """Fase de Inicio: lee credenciales y valida conexión con las 3 APIs. Devuelve creds y estado por API."""
    from modules.evolution.credentials import load_credentials, CREDENTIALS_PATH

    creds = load_credentials()
    logger.info("Credenciales cargadas desde %s", CREDENTIALS_PATH)

    results = {}
    ok_pypi, msg_pypi = _validate_pypi(creds)
    results["pypi"] = {"ok": ok_pypi, "message": msg_pypi}
    logger.info("[Inicio] PyPI: %s", msg_pypi)

    ok_gh, msg_gh = _validate_github(creds)
    results["github"] = {"ok": ok_gh, "message": msg_gh}
    logger.info("[Inicio] GitHub: %s", msg_gh)

    ok_hf, msg_hf = _validate_huggingface(creds)
    results["huggingface"] = {"ok": ok_hf, "message": msg_hf}
    logger.info("[Inicio] Hugging Face: %s", msg_hf)

    return {"creds": creds, "apis": results}


def main() -> None:
    """Punto de entrada: Cerebro (Carga de Energía + Visión Activa) + daemon perpetuo cada 12h."""
    from modules.evolution.credentials import CREDENTIALS_PATH
    if not CREDENTIALS_PATH.exists():
        logger.warning("Fuente de Poder no encontrada: %s — Tokens PyPI/GitHub/HF obligatorios para producción.", CREDENTIALS_PATH)
    logger.info("ATLAS_EVOLUTION Daemon — Cerebro: Carga de Energía + Visión Activa | Sandbox: %s", TEMP_GROWTH)

    # Carga de Energía: API Keys obligatorias desde C:\\dev\\credenciales.txt y validación 3 APIs
    estado = fase_inicio()
    creds = estado["creds"]
    apis = estado["apis"]

    # Crear sandbox temp_growth
    TEMP_GROWTH.mkdir(parents=True, exist_ok=True)
    TEMP_GROWTH_VENV.mkdir(parents=True, exist_ok=True)
    TEMP_GROWTH_WORKSPACE.mkdir(parents=True, exist_ok=True)
    TEMP_GROWTH_MODELS.mkdir(parents=True, exist_ok=True)

    # Inyectar rutas sandbox y intervalo 12h en el daemon
    import evolution_daemon as ed
    ed.TEMP_VENV = TEMP_GROWTH_VENV
    ed.TEMP_WORKSPACE = TEMP_GROWTH_WORKSPACE
    ed.TEMP_MODELS_CACHE = TEMP_GROWTH_MODELS
    ed.SCAN_INTERVAL_SEC = int(os.environ.get("ATLAS_EVOLUTION_INTERVAL_SEC", str(SCAN_INTERVAL_12H)))
    ed.BASE = BASE

    daemon = ed.AtlasEvolutionDaemon()
    logger.info("Ciclo de Crecimiento cada %s s (12 h). Workers: PyPI | GitHub | Hugging Face", ed.SCAN_INTERVAL_SEC)
    try:
        asyncio.run(daemon.run_forever())
    except KeyboardInterrupt:
        logger.info("ATLAS_EVOLUTION detenido por usuario")


if __name__ == "__main__":
    main()
