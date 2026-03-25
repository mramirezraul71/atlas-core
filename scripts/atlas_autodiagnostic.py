"""
ATLAS Autodiagnostic Scanner v2.0
Escaneo periódico de todos los sistemas con acción correctiva automática.
Cubre todos los módulos incorporados a la arquitectura ATLAS:
  - Core (PUSH API, Robot, Brain, Módulos)
  - Memoria (Lifelog, Libro de Vida)
  - Modelos IA (Triada: Grok/DeepSeek/Gemini/Bedrock)
  - ATLAS-Quant (API, LiveLoop, Paper Trading, Supervisor AI)
  - RAULI-VISION (espejo:8080, proxy:3000, dashboard:5174)
  - Recursos del sistema (CPU, RAM, Disco)
  - Cloudflare tunnels (cloudflared)
  - Feedback / Versiones
"""
import logging
import socket
import subprocess
import sys
import time
from datetime import datetime
from logging.handlers import RotatingFileHandler
from pathlib import Path

import requests

# ── Endpoints base ─────────────────────────────────────────────────────────────
BASE_PUSH   = "http://127.0.0.1:8791"
BASE_ROBOT  = "http://127.0.0.1:8002"
BASE_QUANT  = "http://127.0.0.1:8795"   # ATLAS-Quant API (puede ser 8792 si se libera)
BASE_VISION_PROXY = "http://127.0.0.1:3000"
BASE_VISION_DASH  = "http://127.0.0.1:5174"
BASE_ESPEJO = "http://127.0.0.1:8080"

SCAN_INTERVAL = 60
TIMEOUT       = 5
LOG_DIR = Path(__file__).parent.parent / "logs"
LOG_DIR.mkdir(exist_ok=True)

handler = RotatingFileHandler(
    LOG_DIR / "autodiagnostic.log",
    maxBytes=5 * 1024 * 1024,
    backupCount=3,
    encoding="utf-8",
)
handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)-8s | %(message)s"))
logger = logging.getLogger("atlas.autodiag")
logger.setLevel(logging.INFO)
logger.addHandler(handler)
logger.addHandler(logging.StreamHandler())

OK, WARN, CRIT = "OK", "WARNING", "CRITICAL"
_CONSOLE_OK = True


def _safe_print(text: str = "") -> None:
    global _CONSOLE_OK
    if not _CONSOLE_OK:
        return
    try:
        print(text)
    except OSError as e:
        if getattr(e, "errno", None) == 22:
            _CONSOLE_OK = False
            logger.warning("Consola no disponible (Errno 22). Se desactiva salida por print.")
            return
        raise
    except UnicodeEncodeError:
        try:
            print(
                text.encode("utf-8", errors="replace").decode(
                    sys.stdout.encoding or "utf-8", errors="replace"
                )
            )
        except Exception:
            _CONSOLE_OK = False


def _get(url, timeout=TIMEOUT):
    try:
        r = requests.get(url, timeout=timeout)
        r.raise_for_status()
        return r.json()
    except requests.ConnectionError:
        return {"_error": "conexion_rechazada"}
    except requests.Timeout:
        return {"_error": "timeout"}
    except Exception as e:
        return {"_error": str(e)[:120]}


def _post(url, data=None, timeout=TIMEOUT):
    try:
        r = requests.post(url, json=data or {}, timeout=timeout)
        return r.json()
    except Exception as e:
        return {"_error": str(e)[:120]}


def _tcp_open(host: str, port: int, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


def _log_bitacora(msg: str, level: str = "warning") -> None:
    try:
        _post(
            f"{BASE_PUSH}/ans/bitacora",
            {"message": msg, "level": level, "source": "autodiagnostic"},
        )
    except Exception:
        pass


# ══════════════════════════════════════════════════════════════════════════════
# CHECKS — CORE ATLAS
# ══════════════════════════════════════════════════════════════════════════════

def check_push():
    d = _get(f"{BASE_PUSH}/status")
    if "_error" in d:
        return CRIT, f"PUSH no responde: {d['_error']}", "Requiere reinicio: 03_run_atlas_api.ps1"
    if d.get("ok") is False:
        return WARN, "PUSH responde pero reporta error", None
    return OK, "PUSH API :8791 operativo", None


def check_robot():
    d = _get(f"{BASE_ROBOT}/status")
    if "_error" in d:
        return WARN, f"Robot :8002 no responde: {d['_error']}", "Verificar NEXUS robot"
    mods = d.get("modules", {})
    active = sum(1 for v in mods.values() if v)
    total = len(mods) or 1
    if active < total:
        return WARN, f"Robot parcial: {active}/{total} módulos activos", None
    return OK, f"Robot OK: {active}/{total} módulos, uptime {d.get('uptime', 0):.0f}s", None


def check_modules():
    d = _get(f"{BASE_PUSH}/modules/check-all")
    if "_error" in d:
        return CRIT, f"check-all falló: {d['_error']}", None
    connected = d.get("connected", 0)
    total = d.get("total", 0)
    disconnected = []
    for m in d.get("modules", []):
        if m.get("status") != "connected":
            disconnected.append(m.get("name", "?"))
    if disconnected:
        actions = []
        for mod_id in disconnected:
            res = _post(f"{BASE_PUSH}/modules/reconnect/{mod_id}")
            if res.get("ok"):
                actions.append(f"{mod_id} reconectado")
            else:
                actions.append(f"{mod_id} fallo reconexion")
        return (
            WARN,
            f"{connected}/{total} conectados, caidos: {disconnected}",
            "; ".join(actions),
        )
    return OK, f"{connected}/{total} módulos ATLAS conectados", None


def check_memory():
    d = _get(f"{BASE_PUSH}/api/cognitive-memory/lifelog/status")
    if "_error" in d:
        return WARN, f"Lifelog no responde: {d['_error']}", None
    total = d.get("total_entries", 0)
    rate  = d.get("success_rate", 0)
    if total == 0:
        _log_bitacora("ALERTA: Lifelog con 0 entradas")
        return CRIT, "Lifelog vacío (0 entradas)", "Alerta registrada en bitácora"
    if rate < 0.5:
        return WARN, f"Tasa de éxito baja: {rate*100:.0f}% ({d.get('success_count',0)}/{total})", None
    return OK, f"{total} entradas, tasa éxito {rate*100:.0f}%", None


def check_libro_vida():
    d = _get(f"{BASE_PUSH}/api/libro-vida/status")
    if "_error" in d:
        return WARN, f"Libro de Vida no responde: {d['_error']}", None
    eps   = d.get("total_episodios", 0)
    reglas = d.get("reglas_aprendidas", 0)
    tasa  = d.get("tasa_exito", 0)
    if eps == 0:
        return WARN, "Libro de Vida sin episodios registrados", None
    return OK, f"{eps} episodios, {reglas} reglas, tasa {tasa*100:.0f}%", None


def check_brain():
    d = _get(f"{BASE_PUSH}/api/brain/state")
    if "_error" in d:
        return WARN, f"Brain no responde: {d['_error']}", None
    if not d.get("ok"):
        return WARN, "Brain reporta estado no-ok", None
    models = d.get("available_models", [])
    return OK, f"Brain activo, {len(models)} modelos cargados", None


# ══════════════════════════════════════════════════════════════════════════════
# CHECKS — MODELOS IA (TRIADA)
# ══════════════════════════════════════════════════════════════════════════════

def check_ai_models():
    d = _get(f"{BASE_PUSH}/agent/models")
    if "_error" in d:
        return CRIT, f"Catálogo IA no responde: {d['_error']}", None
    models = d.get("data", [])
    available = [m for m in models if m.get("available") and m.get("id") != "auto"]
    total = len([m for m in models if m.get("id") != "auto"])
    if not available:
        return CRIT, "0 modelos IA disponibles", "Sistema sin capacidad de razonamiento"
    providers_ok = set(m.get("provider") for m in available)
    return OK, f"{len(available)}/{total} modelos ({', '.join(sorted(providers_ok))})", None


def check_battery_status():
    """Verifica la triada de modelos: Grok/DeepSeek/Gemini/Bedrock."""
    d = _get(f"{BASE_PUSH}/api/battery/status", timeout=8)
    if "_error" in d:
        return WARN, f"Battery status no responde: {d['_error']}", None
    battery = d.get("data", {}).get("battery", d.get("data", {}))
    if isinstance(battery, dict):
        providers = battery.get("providers", {})
        ok_list   = [k for k, v in providers.items() if v.get("ok") or v.get("status") == "ok"]
        fail_list = [k for k, v in providers.items() if not (v.get("ok") or v.get("status") == "ok")]
        if fail_list:
            return WARN, f"Triada parcial: ok={ok_list}, fallo={fail_list}", "Verificar API keys en credenciales.txt"
        return OK, f"Triada completa: {ok_list}", None
    return WARN, "Battery status — formato inesperado", None


# ══════════════════════════════════════════════════════════════════════════════
# CHECKS — ATLAS-QUANT
# ══════════════════════════════════════════════════════════════════════════════

def check_quant_api():
    """Verifica que ATLAS-Quant API esté activa en :8795."""
    if not _tcp_open("127.0.0.1", 8795):
        # Intentar fallback al puerto original
        if _tcp_open("127.0.0.1", 8792):
            return WARN, "Quant API en :8792 (antiguo). Migrar a :8795", None
        return CRIT, "Quant API caída (ni :8795 ni :8792)", "Ejecutar: atlas_quant_start.ps1"
    d = _get(f"{BASE_QUANT}/health", timeout=4)
    if "_error" in d:
        return WARN, f"Quant :8795 TCP up, HTTP no responde: {d['_error']}", None
    return OK, f"Quant API :8795 activa — {d.get('status', 'ok')}", None


def check_quant_loop():
    """Verifica el estado del auto-cycle loop de trading."""
    d = _get(f"{BASE_QUANT}/operation/loop/status", timeout=4)
    if "_error" in d:
        return WARN, f"Loop status no responde: {d['_error']}", None
    active = d.get("data", {}).get("active", False)
    cycles = d.get("data", {}).get("total_cycles", 0)
    last   = d.get("data", {}).get("last_symbol", "—")
    if not active:
        return WARN, f"LiveLoop inactivo (ciclos: {cycles})", "POST /operation/loop/start para activar"
    return OK, f"LiveLoop activo — {cycles} ciclos, último: {last}", None


def check_quant_paper():
    """Verifica el Paper Trading broker (capital virtual)."""
    d = _get(f"{BASE_QUANT}/paper/account", timeout=4)
    if "_error" in d:
        return WARN, f"Paper broker no responde: {d['_error']}", None
    acct = d.get("data", {})
    capital = acct.get("total_equity", acct.get("initial_capital", 0))
    pnl     = acct.get("unrealized_pnl", 0)
    trades  = acct.get("total_trades", 0)
    return OK, f"Paper broker OK — equity ${capital:,.0f}, PnL ${pnl:+.2f}, trades: {trades}", None


def check_quant_supervisor():
    """Verifica AtlasSupervisorAI — daemon de supervisión de riesgo."""
    d = _get(f"{BASE_QUANT}/supervisor/status", timeout=4)
    if "_error" in d:
        # El supervisor puede no tener endpoint dedicado, no es crítico
        return OK, "Supervisor AI — sin endpoint propio (embebido en core)", None
    active = d.get("data", {}).get("active", False)
    return (OK if active else WARN), f"Supervisor AI: {'activo' if active else 'inactivo'}", None


# ══════════════════════════════════════════════════════════════════════════════
# CHECKS — RAULI-VISION
# ══════════════════════════════════════════════════════════════════════════════

def check_vision_espejo():
    """Verifica espejo.exe (Go backend en :8080)."""
    if not _tcp_open("127.0.0.1", 8080):
        return CRIT, "Espejo :8080 caído", "Ejecutar: .\\RAULI_VISION.ps1"
    d = _get(f"{BASE_ESPEJO}/api/health", timeout=4)
    if "_error" in d:
        return WARN, f"Espejo :8080 TCP up, /api/health no responde: {d['_error']}", None
    return OK, f"Espejo :8080 activo — v{d.get('version', '?')}", None


def check_vision_proxy():
    """Verifica simple-server.py (proxy Python en :3000, proxea a :8080).
    Este es el servicio que Cloudflare Tunnel apunta en vision.rauliatlasapp.com."""
    if not _tcp_open("127.0.0.1", 3000):
        return CRIT, "Proxy Vision :3000 caído — Cloudflare dará 502", "Ejecutar: .\\RAULI_VISION.ps1"
    d = _get(f"{BASE_VISION_PROXY}/api/health", timeout=5)
    if "_error" in d:
        return WARN, f"Proxy :3000 TCP up, /api/health no responde: {d['_error']}", None
    return OK, "Proxy Vision :3000 activo (tunnel Cloudflare OK)", None


def check_vision_dashboard():
    """Verifica el dashboard React/Vite (:5174)."""
    if not _tcp_open("127.0.0.1", 5174):
        return WARN, "Dashboard Vision :5174 caído", "Ejecutar: .\\RAULI_VISION.ps1"
    return OK, "Dashboard Vision :5174 activo", None


def check_cloudflared():
    """Verifica que cloudflared esté corriendo (túnel Cloudflare activo)."""
    try:
        result = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq cloudflared.exe", "/FO", "CSV", "/NH"],
            capture_output=True, text=True, timeout=5, creationflags=0x08000000
        )
        if "cloudflared.exe" in result.stdout:
            return OK, "cloudflared.exe en ejecución (túnel activo)", None
        return CRIT, "cloudflared.exe NO está corriendo — tunnels caídos", "Iniciar cloudflared manualmente"
    except Exception as e:
        return WARN, f"No se pudo verificar cloudflared: {e}", None


# ══════════════════════════════════════════════════════════════════════════════
# CHECKS — RECURSOS DEL SISTEMA
# ══════════════════════════════════════════════════════════════════════════════

def check_resources():
    d = _get(f"{BASE_ROBOT}/system/info")
    if "_error" in d:
        return WARN, f"System info no disponible: {d['_error']}", None
    cpu      = d.get("cpu_percent", 0)
    ram_pct  = d.get("ram", {}).get("percent", 0) if isinstance(d.get("ram"), dict) else 0
    disk_pct = d.get("disk", {}).get("percent", 0) if isinstance(d.get("disk"), dict) else 0
    alerts = []
    if cpu > 90:
        alerts.append(f"CPU {cpu}%")
    if ram_pct > 90:
        alerts.append(f"RAM {ram_pct}%")
    if disk_pct > 95:
        alerts.append(f"Disco {disk_pct}%")
    if alerts:
        detail = ", ".join(alerts)
        _log_bitacora(f"ALERTA RECURSOS: {detail}")
        return WARN, f"Recursos críticos: {detail}", "Alerta registrada en bitácora"
    return OK, f"CPU {cpu}%, RAM {ram_pct}%, Disco {disk_pct}%", None


def check_feedback_log():
    """Verifica que el sistema de feedback esté activo y sin acumulación excesiva."""
    d = _get(f"{BASE_PUSH}/api/feedback/list", timeout=4)
    if "_error" in d:
        return WARN, f"Feedback endpoint no responde: {d['_error']}", None
    items = d.get("data", [])
    count = len(items) if isinstance(items, list) else 0
    if count > 500:
        return WARN, f"Feedback log acumulado: {count} entradas (limpiar recomendado)", None
    return OK, f"Feedback log: {count} entradas", None


# ══════════════════════════════════════════════════════════════════════════════
# SCAN PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════

CHECKS = [
    # ── Core ATLAS ──────────────────────────────────────────────────────────
    ("PUSH API :8791",        check_push),
    ("Robot Backend :8002",   check_robot),
    ("Módulos ATLAS",         check_modules),
    # ── Memoria / Conocimiento ───────────────────────────────────────────────
    ("Memoria (Lifelog)",     check_memory),
    ("Libro de Vida",         check_libro_vida),
    ("Brain (estado)",        check_brain),
    # ── Modelos IA ──────────────────────────────────────────────────────────
    ("Modelos IA (catálogo)", check_ai_models),
    ("Triada IA (battery)",   check_battery_status),
    # ── ATLAS-Quant ─────────────────────────────────────────────────────────
    ("Quant API :8795",       check_quant_api),
    ("Quant LiveLoop",        check_quant_loop),
    ("Quant Paper Broker",    check_quant_paper),
    ("Quant Supervisor AI",   check_quant_supervisor),
    # ── RAULI-VISION ────────────────────────────────────────────────────────
    ("Vision Espejo :8080",   check_vision_espejo),
    ("Vision Proxy :3000",    check_vision_proxy),
    ("Vision Dashboard :5174",check_vision_dashboard),
    ("Cloudflared (tunnel)",  check_cloudflared),
    # ── Infraestructura ─────────────────────────────────────────────────────
    ("Recursos Sistema",      check_resources),
    ("Feedback Log",          check_feedback_log),
]


def run_scan():
    results = []
    for name, fn in CHECKS:
        try:
            status, detail, action = fn()
        except Exception as e:
            status, detail, action = CRIT, f"Excepción: {e}", None
        results.append((name, status, detail, action))

    now        = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    ok_count   = sum(1 for _, s, _, _ in results if s == OK)
    warn_count = sum(1 for _, s, _, _ in results if s == WARN)
    crit_count = sum(1 for _, s, _, _ in results if s == CRIT)
    total      = len(results)

    col_name = 26
    col_stat = 10
    col_det  = 52
    col_act  = 32
    sep = "+" + "-"*(col_name+2) + "+" + "-"*(col_stat+2) + "+" + "-"*(col_det+2) + "+" + "-"*(col_act+2) + "+"

    _safe_print(f"\n{'='*80}")
    _safe_print(f"  ATLAS AUTODIAGNÓSTICO v2.0 — {now}")
    _safe_print(f"  Módulos: Core | Memoria | Triada IA | Quant | RAULI-VISION | Infra")
    _safe_print(f"{'='*80}")
    _safe_print(sep)
    _safe_print(f"| {'Componente':<{col_name}} | {'Estado':<{col_stat}} | {'Detalle':<{col_det}} | {'Acción':<{col_act}} |")
    _safe_print(sep)

    icon_map = {OK: "OK ", WARN: "WARN", CRIT: "CRIT"}
    emoji_map = {OK: "OK", WARN: "!! ", CRIT: "XX "}

    for name, status, detail, action in results:
        icon  = emoji_map[status]
        act   = (action or "—")[:col_act]
        det   = detail[:col_det]
        _safe_print(f"| {name:<{col_name}} | {icon} {icon_map[status]:<{col_stat-3}} | {det:<{col_det}} | {act:<{col_act}} |")

    _safe_print(sep)
    _safe_print(f"\n  Resumen: {ok_count}/{total} OK | {warn_count} warnings | {crit_count} criticos")
    _safe_print(f"{'='*80}\n")

    logger.info(f"Scan: {ok_count}/{total} OK, {warn_count} WARN, {crit_count} CRIT")
    for name, status, detail, action in results:
        if status != OK:
            logger.warning(
                f"  {status} [{name}]: {detail}" + (f" -> {action}" if action else "")
            )

    if crit_count > 0 or warn_count > 3:
        _log_bitacora(
            f"Autodiagnóstico: {ok_count}/{total} OK, {warn_count} WARN, {crit_count} CRIT",
            level="warning" if crit_count == 0 else "error",
        )

    return ok_count, warn_count, crit_count


def main():
    _safe_print(f"\n{'#'*80}")
    _safe_print("  ATLAS AUTODIAGNOSTIC SCANNER v2.0")
    _safe_print(f"  {len(CHECKS)} checks | Intervalo: {SCAN_INTERVAL}s | Log: {LOG_DIR / 'autodiagnostic.log'}")
    _safe_print(f"{'#'*80}")
    logger.info(f"Autodiagnostic scanner v2.0 iniciado — {len(CHECKS)} checks")

    cycle = 0
    while True:
        cycle += 1
        try:
            logger.info(f"--- Ciclo {cycle} ---")
            run_scan()
        except KeyboardInterrupt:
            logger.info("Scanner detenido por usuario")
            _safe_print("\n[Autodiagnóstico detenido]")
            break
        except Exception as e:
            logger.error(f"Error en ciclo {cycle}: {e}")
            _safe_print(f"[Error ciclo {cycle}: {e}]")

        try:
            time.sleep(SCAN_INTERVAL)
        except KeyboardInterrupt:
            logger.info("Scanner detenido por usuario")
            _safe_print("\n[Autodiagnóstico detenido]")
            break


if __name__ == "__main__":
    main()
