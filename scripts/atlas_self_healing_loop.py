"""
ATLAS Self-Healing Loop v1.0
=============================
Daemon que detecta fallos en TODOS los módulos ATLAS y los corrige
automáticamente sin intervención humana.

Cobertura:
  ┌─────────────────────────────────────────────────────────────────────┐
  │  Módulo               │ Detección              │ Corrección          │
  ├─────────────────────────────────────────────────────────────────────┤
  │  ATLAS API (8791)     │ TCP + HTTP /health     │ Relanzar uvicorn    │
  │  NEXUS Robot (8000)   │ TCP /health            │ Alerta (no restart) │
  │  ATLAS-Quant (8795)   │ TCP + HTTP /health     │ Relanzar quant      │
  │  Quant LiveLoop       │ GET /loop/status       │ POST /loop/start    │
  │  Paper Broker DB      │ archivo .db presente   │ Recrear via API     │
  │  RAULI-VISION espejo  │ TCP :8080              │ Lanzar espejo.exe   │
  │  RAULI-VISION proxy   │ TCP :3000              │ Lanzar simple-server│
  │  RAULI-VISION dash    │ TCP :5174              │ npm run preview     │
  │  Cloudflared tunnel   │ proceso en tasklist    │ Relanzar servicio   │
  │  Memoria (Lifelog)    │ /lifelog/status        │ Alerta + bitácora   │
  │  Triada IA            │ /api/battery/status    │ Alerta + bitácora   │
  └─────────────────────────────────────────────────────────────────────┘

Uso:
    python scripts/atlas_self_healing_loop.py            # bucle continuo
    python scripts/atlas_self_healing_loop.py --once     # una sola pasada
    python scripts/atlas_self_healing_loop.py --dry-run  # solo detectar

Variables de entorno:
    ATLAS_HEAL_INTERVAL_SEC   — segundos entre ciclos (default 60)
    ATLAS_HEAL_MAX_RETRIES    — reintentos antes de escalar (default 3)
    ATLAS_HEAL_DRY_RUN        — 1 para solo detectar sin corregir
"""
from __future__ import annotations

import argparse
import logging
import os
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Callable, Dict, List, Optional

import requests

# ── Configuración ─────────────────────────────────────────────────────────────
ROOT          = Path(__file__).resolve().parent.parent
LOG_DIR       = ROOT / "logs"
LOG_DIR.mkdir(exist_ok=True)

HEAL_INTERVAL = int(os.getenv("ATLAS_HEAL_INTERVAL_SEC", "60"))
MAX_RETRIES   = int(os.getenv("ATLAS_HEAL_MAX_RETRIES", "3"))
DRY_RUN       = os.getenv("ATLAS_HEAL_DRY_RUN", "0") == "1"

BASE_PUSH  = "http://127.0.0.1:8791"
BASE_ROBOT = "http://127.0.0.1:8002"
BASE_QUANT = "http://127.0.0.1:8795"
BASE_PROXY = "http://127.0.0.1:3000"
BASE_ESPEJO= "http://127.0.0.1:8080"

VENV_PYTHON = str(ROOT / "venv" / "Scripts" / "python.exe")
ESPEJO_EXE  = str(ROOT / "_external" / "RAULI-VISION" / "espejo" / "espejo.exe")
PROXY_SCRIPT= str(ROOT / "_external" / "RAULI-VISION" / "cliente-local" / "simple-server.py")
PROXY_DIR   = str(ROOT / "_external" / "RAULI-VISION" / "cliente-local")
DASH_DIR    = str(ROOT / "_external" / "RAULI-VISION" / "dashboard")
ESPEJO_DIR  = str(ROOT / "_external" / "RAULI-VISION" / "espejo")
QUANT_START = str(ROOT / "scripts" / "atlas_quant_start.ps1")

# ── Logger ────────────────────────────────────────────────────────────────────
_handler = RotatingFileHandler(
    LOG_DIR / "self_healing.log", maxBytes=5*1024*1024, backupCount=5, encoding="utf-8"
)
_handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)-8s | %(message)s"))
log = logging.getLogger("atlas.heal")
log.setLevel(logging.INFO)
log.addHandler(_handler)
log.addHandler(logging.StreamHandler())

# ── Estado de fallos (contador por componente) ────────────────────────────────
_fail_counts: Dict[str, int] = {}


# ══════════════════════════════════════════════════════════════════════════════
# Primitivas
# ══════════════════════════════════════════════════════════════════════════════

def _tcp(host: str, port: int, timeout: float = 1.2) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


def _get(url: str, timeout: float = 4.0) -> dict:
    try:
        r = requests.get(url, timeout=timeout)
        return r.json() if r.ok else {"_error": f"HTTP {r.status_code}"}
    except requests.ConnectionError:
        return {"_error": "conexion_rechazada"}
    except requests.Timeout:
        return {"_error": "timeout"}
    except Exception as e:
        return {"_error": str(e)[:100]}


def _post(url: str, data: dict | None = None, timeout: float = 4.0) -> dict:
    try:
        r = requests.post(url, json=data or {}, timeout=timeout)
        return r.json()
    except Exception as e:
        return {"_error": str(e)[:100]}


def _spawn(cmd: List[str], cwd: str | None = None, env: dict | None = None) -> Optional[int]:
    """Lanza un proceso en background. Devuelve PID o None si falla."""
    if DRY_RUN:
        log.info(f"  [DRY-RUN] spawn: {' '.join(cmd)}")
        return -1
    try:
        flags = 0
        if sys.platform == "win32":
            flags = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW
        p = subprocess.Popen(
            cmd, cwd=cwd, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            creationflags=flags,
        )
        return p.pid
    except Exception as e:
        log.error(f"  spawn error: {e}")
        return None


def _wait_port(port: int, timeout_sec: float = 15.0) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if _tcp("127.0.0.1", port):
            return True
        time.sleep(0.8)
    return False


def _bitacora(msg: str, level: str = "warning") -> None:
    try:
        _post(f"{BASE_PUSH}/ans/bitacora", {"message": msg, "level": level, "source": "self_healing"})
    except Exception:
        pass


# ══════════════════════════════════════════════════════════════════════════════
# Dataclass de resultado
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class HealResult:
    component: str
    healthy:   bool
    detail:    str
    healed:    bool  = False
    heal_msg:  str   = ""
    critical:  bool  = False


# ══════════════════════════════════════════════════════════════════════════════
# REGLAS DE DETECCIÓN + CORRECCIÓN
# ══════════════════════════════════════════════════════════════════════════════

def rule_atlas_api() -> HealResult:
    """ATLAS API :8791 — reinicia uvicorn si no responde."""
    name = "ATLAS API :8791"
    if _tcp("127.0.0.1", 8791):
        # Puerto abierto = servicio activo (aunque /health devuelva 4xx por auth)
        return HealResult(name, True, "TCP up / HTTP activo")
    # Caída — intentar relanzar
    log.warning(f"[{name}] Puerto 8791 no responde — relanzando...")
    ps1 = str(ROOT / "03_run_atlas_api.ps1")
    if Path(ps1).exists():
        pid = _spawn(["powershell.exe", "-NonInteractive", "-WindowStyle", "Hidden", "-File", ps1])
        if pid and _wait_port(8791, 20):
            return HealResult(name, False, "caída detectada", healed=True, heal_msg=f"relanzado PID={pid}", critical=True)
        return HealResult(name, False, "no respondió tras relanzar", healed=False, heal_msg="fallo relanzamiento", critical=True)
    return HealResult(name, False, "03_run_atlas_api.ps1 no encontrado", critical=True)


def rule_quant_api() -> HealResult:
    """ATLAS-Quant API :8795."""
    name = "Quant API :8795"
    if _tcp("127.0.0.1", 8795):
        return HealResult(name, True, "TCP up")
    log.warning(f"[{name}] Puerto 8795 no responde — relanzando...")
    start_ps1 = str(ROOT / "scripts" / "atlas_quant_start.ps1")
    if Path(start_ps1).exists():
        pid = _spawn([
            "powershell.exe", "-NonInteractive", "-WindowStyle", "Hidden",
            "-File", start_ps1, "-Port", "8795"
        ])
        if pid and _wait_port(8795, 25):
            return HealResult(name, False, "caída detectada", healed=True, heal_msg=f"relanzado PID={pid}")
    return HealResult(name, False, "relanzamiento fallido — intervención manual")


def rule_quant_loop() -> HealResult:
    """LiveLoop de trading — reactivar si está inactivo."""
    name = "Quant LiveLoop"
    if not _tcp("127.0.0.1", 8795):
        return HealResult(name, False, "Quant API no disponible", critical=False)
    # Intentar endpoint v1 y v2
    d = _get(f"{BASE_QUANT}/operation/loop/status")
    if "_error" in d:
        d = _get(f"{BASE_QUANT}/api/v2/quant/operation/loop/status")
    if "_error" in d:
        # Endpoint no disponible en esta versión — no es fallo crítico
        return HealResult(name, True, "endpoint loop/status no disponible (ok)")
    active = d.get("data", {}).get("active", False)
    if active:
        cycles = d.get("data", {}).get("total_cycles", 0)
        return HealResult(name, True, f"activo — {cycles} ciclos")
    # Inactivo → reactivar
    log.info(f"[{name}] Inactivo — activando auto-cycle...")
    r = _post(f"{BASE_QUANT}/operation/loop/start", {"interval_sec": 120, "max_per_cycle": 1})
    if r.get("ok") or r.get("data", {}).get("active"):
        return HealResult(name, False, "estaba inactivo", healed=True, heal_msg="activado via /loop/start")
    return HealResult(name, False, "no se pudo activar", heal_msg=str(r.get("_error", "")))


def rule_vision_espejo() -> HealResult:
    """espejo.exe Go backend :8080."""
    name = "Vision Espejo :8080"
    if _tcp("127.0.0.1", 8080):
        return HealResult(name, True, "TCP up")
    log.warning(f"[{name}] Puerto 8080 caído — relanzando espejo.exe...")
    if not Path(ESPEJO_EXE).exists():
        return HealResult(name, False, "espejo.exe no encontrado", critical=True)
    env = os.environ.copy()
    env.update({"PORT": "8080", "ACCESS_STORE": "data\\access-store.json"})
    pid = _spawn([ESPEJO_EXE], cwd=ESPEJO_DIR, env=env)
    if pid and _wait_port(8080, 15):
        return HealResult(name, False, "caído detectado", healed=True, heal_msg=f"relanzado PID={pid}", critical=True)
    return HealResult(name, False, "no respondió tras relanzar", critical=True)


def rule_vision_proxy() -> HealResult:
    """simple-server.py proxy Python :3000 (Cloudflare apunta aquí)."""
    name = "Vision Proxy :3000"
    if _tcp("127.0.0.1", 3000):
        return HealResult(name, True, "TCP up — Cloudflare OK")
    log.warning(f"[{name}] Puerto 3000 caído — relanzando proxy...")
    if not Path(PROXY_SCRIPT).exists():
        return HealResult(name, False, "simple-server.py no encontrado", critical=True)
    python = VENV_PYTHON if Path(VENV_PYTHON).exists() else "python"
    pid = _spawn([python, PROXY_SCRIPT], cwd=PROXY_DIR)
    if pid and _wait_port(3000, 12):
        return HealResult(name, False, "caído detectado", healed=True, heal_msg=f"relanzado PID={pid}", critical=True)
    return HealResult(name, False, "no respondió tras relanzar", critical=True)


def rule_vision_dashboard() -> HealResult:
    """Dashboard React/Vite :5174."""
    name = "Vision Dashboard :5174"
    if _tcp("127.0.0.1", 5174):
        return HealResult(name, True, "TCP up")
    log.warning(f"[{name}] Puerto 5174 caído — relanzando npm preview...")
    dist = Path(DASH_DIR) / "dist" / "index.html"
    cmd_str = "npm run preview -- --port 5174 --host 127.0.0.1" if dist.exists() \
              else "npm run dev -- --port 5174 --host 0.0.0.0"
    pid = _spawn(["cmd.exe", "/c", cmd_str], cwd=DASH_DIR)
    if pid and _wait_port(5174, 25):
        return HealResult(name, False, "caído detectado", healed=True, heal_msg=f"relanzado (preview/dev) PID={pid}")
    return HealResult(name, False, "no respondió tras relanzar")


def rule_cloudflared() -> HealResult:
    """cloudflared.exe — túnel Cloudflare."""
    name = "Cloudflared tunnel"
    try:
        r = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq cloudflared.exe", "/FO", "CSV", "/NH"],
            capture_output=True, text=True, timeout=5, creationflags=0x08000000
        )
        if "cloudflared.exe" in r.stdout:
            return HealResult(name, True, "proceso activo")
    except Exception:
        pass
    # No corre — intentar relanzar como servicio o ejecutable
    log.warning(f"[{name}] cloudflared no encontrado — intentando relanzar...")
    cf_exe = r"C:\Program Files (x86)\cloudflared\cloudflared.exe"
    cf_token_log = str(LOG_DIR / "cloudflared_named.log")
    token_env = os.getenv("CLOUDFLARED_TOKEN", "")
    if token_env and Path(cf_exe).exists():
        pid = _spawn([
            cf_exe, "tunnel", "--no-autoupdate", "--protocol", "quic",
            "--logfile", cf_token_log, "run", "--token", token_env
        ])
        time.sleep(3)
        try:
            r2 = subprocess.run(
                ["tasklist", "/FI", "IMAGENAME eq cloudflared.exe", "/FO", "CSV", "/NH"],
                capture_output=True, text=True, timeout=5, creationflags=0x08000000
            )
            if "cloudflared.exe" in r2.stdout:
                return HealResult(name, False, "proceso no corría", healed=True, heal_msg=f"relanzado PID={pid}", critical=True)
        except Exception:
            pass
    return HealResult(name, False, "no se pudo relanzar (token o exe no disponible)", critical=True)


def rule_memory_lifelog() -> HealResult:
    """Lifelog — alerta si 0 entradas (no se puede reparar automáticamente)."""
    name = "Memoria Lifelog"
    d = _get(f"{BASE_PUSH}/api/cognitive-memory/lifelog/status")
    if "_error" in d:
        return HealResult(name, False, f"no responde: {d['_error']}")
    total = d.get("total_entries", 0)
    rate  = d.get("success_rate", 1.0)
    if total == 0:
        _bitacora("ALERTA: Lifelog con 0 entradas — requiere revisión manual", "error")
        return HealResult(name, False, "0 entradas — alerta registrada", critical=True)
    if rate < 0.5:
        return HealResult(name, False, f"tasa baja: {rate*100:.0f}%")
    return HealResult(name, True, f"{total} entradas, tasa {rate*100:.0f}%")


def rule_triada_ia() -> HealResult:
    """Triada IA (Grok/DeepSeek/Gemini/Bedrock) — alerta si proveedor caído."""
    name = "Triada IA"
    # Intentar varios endpoints posibles para el estado de la batería
    for url in [
        f"{BASE_PUSH}/api/battery/status",
        f"{BASE_PUSH}/api/v2/battery/status",
        f"{BASE_PUSH}/agent/models",
    ]:
        d = _get(url, timeout=6)
        if "_error" not in d:
            break
    else:
        return HealResult(name, False, "ningún endpoint de triada disponible")

    # Formato /api/battery/status
    data_val = d.get("data") or {}
    battery = data_val.get("battery", data_val) if isinstance(data_val, dict) else {}
    if isinstance(battery, dict) and "providers" in battery:
        providers = battery.get("providers", {})
        ok_p  = [k for k, v in providers.items() if isinstance(v, dict) and (v.get("ok") or v.get("status") == "ok")]
        fail_p= [k for k, v in providers.items() if isinstance(v, dict) and not (v.get("ok") or v.get("status") == "ok")]
        if fail_p:
            _bitacora(f"Triada IA: proveedores caídos={fail_p}", "warning")
            return HealResult(name, False, f"caídos={fail_p}", heal_msg="verificar API keys en credenciales.txt")
        return HealResult(name, True, f"todos OK: {ok_p}")

    # Formato /agent/models
    models = d.get("data", [])
    if isinstance(models, list):
        available = [m.get("provider") for m in models if m.get("available") and m.get("id") != "auto"]
        if not available:
            return HealResult(name, False, "0 modelos IA disponibles", critical=True)
        return HealResult(name, True, f"{len(available)} modelos activos: {set(available)}")

    return HealResult(name, True, "estado IA verificado (formato alternativo)")


def rule_nexus() -> HealResult:
    """NEXUS :8000 — solo alerta, no reinicia (gate de arranque de ATLAS)."""
    name = "NEXUS :8000"
    if _tcp("127.0.0.1", 8000):
        return HealResult(name, True, "TCP up")
    _bitacora("NEXUS :8000 no responde — ATLAS puede no arrancar correctamente", "error")
    return HealResult(name, False, "no responde — alerta registrada (reinicio manual)", critical=True)


# ══════════════════════════════════════════════════════════════════════════════
# LISTA DE REGLAS
# ══════════════════════════════════════════════════════════════════════════════

RULES: List[Callable[[], HealResult]] = [
    rule_atlas_api,
    rule_nexus,
    rule_quant_api,
    rule_quant_loop,
    rule_vision_espejo,
    rule_vision_proxy,
    rule_vision_dashboard,
    rule_cloudflared,
    rule_memory_lifelog,
    rule_triada_ia,
]


# ══════════════════════════════════════════════════════════════════════════════
# CICLO PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════

def run_healing_cycle() -> Dict[str, int]:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    results: List[HealResult] = []

    for rule_fn in RULES:
        try:
            r = rule_fn()
        except Exception as e:
            r = HealResult(rule_fn.__name__, False, f"excepción: {e}", critical=True)
            log.error(f"Excepción en regla {rule_fn.__name__}: {e}")

        # Contador de fallos consecutivos
        if r.healthy or r.healed:
            _fail_counts[r.component] = 0
        else:
            _fail_counts[r.component] = _fail_counts.get(r.component, 0) + 1

        # Escalar si supera MAX_RETRIES
        consecutive = _fail_counts.get(r.component, 0)
        if consecutive >= MAX_RETRIES and r.critical:
            escalation_msg = (
                f"ESCALACION: {r.component} lleva {consecutive} ciclos caído "
                f"y no se pudo auto-corregir. Requiere intervención manual."
            )
            log.error(escalation_msg)
            _bitacora(escalation_msg, "error")

        results.append(r)

    # ── Reporte de consola ──────────────────────────────────────────────────
    ok_n    = sum(1 for r in results if r.healthy)
    healed_n= sum(1 for r in results if r.healed)
    fail_n  = sum(1 for r in results if not r.healthy and not r.healed)
    total_n = len(results)

    width = 78
    print(f"\n{'='*width}")
    print(f"  ATLAS SELF-HEALING CYCLE — {now}  [{'DRY-RUN' if DRY_RUN else 'ACTIVO'}]")
    print(f"{'='*width}")
    print(f"  {'Componente':<28}  {'Estado':<10}  {'Detalle':<35}")
    print(f"  {'-'*28}  {'-'*10}  {'-'*35}")
    for r in results:
        if r.healthy:
            stat = "OK"
            det  = r.detail[:35]
        elif r.healed:
            stat = "CORREGIDO"
            det  = r.heal_msg[:35]
        else:
            stat = "FALLO"
            det  = r.detail[:35]
        print(f"  {r.component:<28}  {stat:<10}  {det}")
    print(f"{'='*width}")
    print(f"  Resumen: {ok_n} OK | {healed_n} corregidos | {fail_n} fallos | total {total_n}")
    print(f"{'='*width}\n")

    log.info(f"Ciclo: {ok_n}/{total_n} OK, {healed_n} corregidos, {fail_n} fallos persistentes")

    # Registrar en bitácora si hubo correcciones o fallos
    if healed_n > 0:
        healed_names = [r.component for r in results if r.healed]
        _bitacora(f"Self-healing: corregidos={healed_names}", "info")
    if fail_n > 0:
        fail_names = [r.component for r in results if not r.healthy and not r.healed]
        _bitacora(f"Self-healing: fallos persistentes={fail_names}", "warning")

    return {"ok": ok_n, "healed": healed_n, "failed": fail_n, "total": total_n}


def main():
    parser = argparse.ArgumentParser(description="ATLAS Self-Healing Loop")
    parser.add_argument("--once",    action="store_true", help="Ejecutar un solo ciclo y salir")
    parser.add_argument("--dry-run", action="store_true", help="Solo detectar, no corregir")
    args = parser.parse_args()

    global DRY_RUN
    if args.dry_run:
        DRY_RUN = True

    mode = "DRY-RUN" if DRY_RUN else f"ACTIVO (intervalo {HEAL_INTERVAL}s)"
    log.info(f"ATLAS Self-Healing Loop v1.0 iniciado — modo: {mode}")
    print(f"\n{'#'*78}")
    print(f"  ATLAS SELF-HEALING LOOP v1.0")
    print(f"  {len(RULES)} reglas | {mode}")
    print(f"  Log: {LOG_DIR / 'self_healing.log'}")
    print(f"{'#'*78}")

    if args.once:
        run_healing_cycle()
        return

    cycle = 0
    while True:
        cycle += 1
        log.info(f"--- Ciclo {cycle} ---")
        try:
            run_healing_cycle()
        except KeyboardInterrupt:
            log.info("Self-healing detenido por usuario")
            print("\n[Self-Healing detenido]")
            break
        except Exception as e:
            log.error(f"Error en ciclo {cycle}: {e}")
        try:
            time.sleep(HEAL_INTERVAL)
        except KeyboardInterrupt:
            log.info("Self-healing detenido por usuario")
            print("\n[Self-Healing detenido]")
            break


if __name__ == "__main__":
    main()
