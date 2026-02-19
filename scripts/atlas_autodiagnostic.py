"""
ATLAS Autodiagnostic Scanner
Escaneo periódico de todos los sistemas con acción correctiva automática.
Generado para ATLAS Cognitive Brain Architecture.
"""
import requests
import time
import json
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime
from pathlib import Path

BASE_PUSH = "http://127.0.0.1:8791"
BASE_ROBOT = "http://127.0.0.1:8002"
SCAN_INTERVAL = 60
TIMEOUT = 5
LOG_DIR = Path(__file__).parent.parent / "logs"
LOG_DIR.mkdir(exist_ok=True)

handler = RotatingFileHandler(LOG_DIR / "autodiagnostic.log", maxBytes=5*1024*1024, backupCount=3, encoding="utf-8")
handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)-8s | %(message)s"))
logger = logging.getLogger("atlas.autodiag")
logger.setLevel(logging.INFO)
logger.addHandler(handler)
logger.addHandler(logging.StreamHandler())

OK, WARN, CRIT = "OK", "WARNING", "CRITICAL"


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


def check_push():
    d = _get(f"{BASE_PUSH}/status")
    if "_error" in d:
        return CRIT, f"PUSH no responde: {d['_error']}", "Requiere reinicio manual"
    if d.get("ok") is False:
        return WARN, "PUSH responde pero reporta error", None
    return OK, "PUSH operativo", None


def check_robot():
    d = _get(f"{BASE_ROBOT}/status")
    if "_error" in d:
        return WARN, f"Robot no responde: {d['_error']}", "Registrando incidente"
    mods = d.get("modules", {})
    active = sum(1 for v in mods.values() if v)
    total = len(mods) or 1
    if active < total:
        return WARN, f"Robot parcial: {active}/{total} módulos", None
    return OK, f"Robot OK: {active}/{total} módulos, uptime {d.get('uptime',0):.0f}s", None


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
                actions.append(f"{mod_id} fallo reconexión")
        return WARN, f"{connected}/{total} conectados, desconectados: {disconnected}", "; ".join(actions)
    return OK, f"{connected}/{total} módulos conectados", None


def check_memory():
    d = _get(f"{BASE_PUSH}/api/cognitive-memory/lifelog/status")
    if "_error" in d:
        return WARN, f"Lifelog no responde: {d['_error']}", None
    total = d.get("total_entries", 0)
    success = d.get("success_count", 0)
    fail = d.get("failure_count", 0)
    rate = d.get("success_rate", 0)
    if total == 0:
        _post(f"{BASE_PUSH}/ans/bitacora", {"message": "ALERTA: Lifelog con 0 entradas", "level": "warning", "source": "autodiagnostic"})
        return CRIT, "Lifelog vacío (0 entradas)", "Alerta registrada en bitácora"
    if rate < 0.5:
        return WARN, f"Tasa de éxito baja: {rate*100:.0f}% ({success}/{total})", None
    return OK, f"{total} entradas, tasa éxito {rate*100:.0f}%", None


def check_libro_vida():
    d = _get(f"{BASE_PUSH}/api/libro-vida/status")
    if "_error" in d:
        return WARN, f"Libro de Vida no responde: {d['_error']}", None
    eps = d.get("total_episodios", 0)
    reglas = d.get("reglas_aprendidas", 0)
    tasa = d.get("tasa_exito", 0)
    if eps == 0:
        return WARN, "Libro de Vida sin episodios", None
    return OK, f"{eps} episodios, {reglas} reglas, tasa {tasa*100:.0f}%", None


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
    return OK, f"{len(available)}/{total} modelos disponibles ({', '.join(sorted(providers_ok))})", None


def check_brain():
    d = _get(f"{BASE_PUSH}/api/brain/state")
    if "_error" in d:
        return WARN, f"Brain no responde: {d['_error']}", None
    if not d.get("ok"):
        return WARN, "Brain reporta estado no-ok", None
    models = d.get("available_models", [])
    return OK, f"Brain activo, {len(models)} modelos cargados", None


def check_resources():
    d = _get(f"{BASE_ROBOT}/system/info")
    if "_error" in d:
        return WARN, f"System info no disponible: {d['_error']}", None
    cpu = d.get("cpu_percent", 0)
    ram_pct = d.get("ram", {}).get("percent", 0) if isinstance(d.get("ram"), dict) else 0
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
        _post(f"{BASE_PUSH}/ans/bitacora", {"message": f"ALERTA RECURSOS: {detail}", "level": "warning", "source": "autodiagnostic"})
        return WARN, f"Recursos críticos: {detail}", "Alerta registrada"
    return OK, f"CPU {cpu}%, RAM {ram_pct}%, Disco {disk_pct}%", None


def run_scan():
    checks = [
        ("PUSH Server", check_push),
        ("Robot Backend", check_robot),
        ("Módulos ATLAS", check_modules),
        ("Memoria (Lifelog)", check_memory),
        ("Libro de Vida", check_libro_vida),
        ("Modelos IA", check_ai_models),
        ("Cerebro (Brain)", check_brain),
        ("Recursos Sistema", check_resources),
    ]
    results = []
    for name, fn in checks:
        try:
            status, detail, action = fn()
        except Exception as e:
            status, detail, action = CRIT, f"Excepción: {e}", None
        results.append((name, status, detail, action))

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    ok_count = sum(1 for _, s, _, _ in results if s == OK)
    warn_count = sum(1 for _, s, _, _ in results if s == WARN)
    crit_count = sum(1 for _, s, _, _ in results if s == CRIT)
    total = len(results)

    sep = "+" + "-"*22 + "+" + "-"*12 + "+" + "-"*52 + "+" + "-"*30 + "+"
    header = f"| {'Componente':<20} | {'Estado':<10} | {'Detalle':<50} | {'Acción':<28} |"

    print(f"\n{'='*70}")
    print(f"  ATLAS AUTODIAGNÓSTICO — {now}")
    print(f"{'='*70}")
    print(sep)
    print(header)
    print(sep)
    for name, status, detail, action in results:
        icon = {"OK": "✅", "WARNING": "⚠️ ", "CRITICAL": "❌"}[status]
        act = (action or "—")[:28]
        print(f"| {name:<20} | {icon} {status:<7} | {detail[:50]:<50} | {act:<28} |")
    print(sep)
    print(f"\n  Resumen: {ok_count}/{total} OK | {warn_count} warnings | {crit_count} críticos")
    print(f"{'='*70}\n")

    logger.info(f"Scan: {ok_count}/{total} OK, {warn_count} WARN, {crit_count} CRIT")
    for name, status, detail, action in results:
        if status != OK:
            logger.warning(f"  {status} {name}: {detail}" + (f" -> {action}" if action else ""))

    if crit_count > 0 or warn_count > 2:
        try:
            _post(f"{BASE_PUSH}/ans/bitacora", {
                "message": f"Autodiagnóstico: {ok_count}/{total} OK, {warn_count} WARN, {crit_count} CRIT",
                "level": "warning" if crit_count == 0 else "error",
                "source": "autodiagnostic"
            })
        except:
            pass

    return ok_count, warn_count, crit_count


def main():
    print(f"\n{'#'*70}")
    print(f"  ATLAS AUTODIAGNOSTIC SCANNER")
    print(f"  Intervalo: {SCAN_INTERVAL}s | Log: {LOG_DIR / 'autodiagnostic.log'}")
    print(f"{'#'*70}")
    logger.info("Autodiagnostic scanner iniciado")

    cycle = 0
    while True:
        cycle += 1
        try:
            logger.info(f"--- Ciclo {cycle} ---")
            run_scan()
        except KeyboardInterrupt:
            logger.info("Scanner detenido por usuario")
            print("\n[Autodiagnóstico detenido]")
            break
        except Exception as e:
            logger.error(f"Error en ciclo {cycle}: {e}")
            print(f"[Error ciclo {cycle}: {e}]")

        try:
            time.sleep(SCAN_INTERVAL)
        except KeyboardInterrupt:
            logger.info("Scanner detenido por usuario")
            print("\n[Autodiagnóstico detenido]")
            break


if __name__ == "__main__":
    main()
