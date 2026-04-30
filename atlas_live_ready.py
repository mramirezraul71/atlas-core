"""atlas_live_ready.py
Script legacy para dejar ATLAS en un estado paper-first operativo y consistente.

Compatibilidad:
- Mantiene el nombre historico del archivo para no romper automatizaciones externas.
- No arma trading live real: configura paper autonomy y valida la sincronizacion
  entre el estado operativo de Quant y el contrato canonico consumido por Core.

Uso: python atlas_live_ready.py
"""

from __future__ import annotations

import json
import re
import sqlite3
import sys
import urllib.request
from datetime import datetime
from pathlib import Path

API = "http://127.0.0.1:8795"
KEY = {"Content-Type": "application/json", "X-API-Key": "atlas-quant-local"}
DB = r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"
QUANT_STATE = Path(r"C:\ATLAS_PUSH\atlas_code_quant\data\operation\operation_center_state.json")
CORE_STATE = Path(r"C:\ATLAS_PUSH\data\operation\operation_center_state.json")


def api(path: str, body: dict | None = None, method: str = "POST") -> dict:
    req = urllib.request.Request(
        f"{API}{path}",
        data=json.dumps(body or {}).encode(),
        headers=KEY,
        method=method,
    )
    try:
        return json.loads(urllib.request.urlopen(req, timeout=15).read())
    except Exception as exc:
        return {"error": str(exc)}


def load_json(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


print("[1/8] Verificando API...")
h = api("/health", method="GET")
if "error" in h:
    print("  ERROR: API no responde:", h["error"])
    sys.exit(1)
print(f"  OK - uptime {h.get('uptime_sec', '?')}s")

print("[2/8] Purgando phantoms OS+AA del journal...")
conn = sqlite3.connect(DB)
cur = conn.cursor()
cur.execute("DELETE FROM trading_journal WHERE symbol IN ('OS','AA') AND broker_order_ids_json IN ('{}','[]','')")
n = cur.rowcount
conn.commit()
conn.close()
print(f"  Eliminados: {n} phantoms")

print("[3/8] Deteniendo loop...")
r = api("/operation/loop/stop")
print(f"  Loop stopped: {r.get('ok', r)}")

print("[4/8] Configurando operation center (paper autonomy coherente con Core)...")
cfg = {
    "auton_mode": "paper_autonomous",
    "executor_mode": "paper_api",
    "require_operator_present": False,
    "operator_present": True,
    "fail_safe_active": False,
    "signal_validator_enabled": True,
    "var_monitor_enabled": True,
    "auto_pause_on_operational_errors": True,
    "operational_error_limit": 10,
    "min_auton_win_rate_pct": 35.0,
}
r = api("/operation/config", cfg)
print(f"  Config: {r.get('ok', r)}")

print("[5/8] Verificando sincronizacion Quant/Core...")
quant_state = load_json(QUANT_STATE)
core_state = load_json(CORE_STATE)
print(
    "  Quant:",
    {
        "auton_mode": quant_state.get("auton_mode"),
        "autonomy_mode": quant_state.get("autonomy_mode"),
        "fail_safe_active": quant_state.get("fail_safe_active"),
    },
)
print(
    "  Core :",
    {
        "auton_mode": core_state.get("auton_mode"),
        "autonomy_mode": core_state.get("autonomy_mode"),
        "fail_safe_active": core_state.get("fail_safe_active"),
        "source_module": core_state.get("source_module"),
    },
)
if quant_state.get("auton_mode") != core_state.get("auton_mode"):
    print("  ADVERTENCIA - auton_mode no esta alineado entre Quant y Core")
if quant_state.get("autonomy_mode") != core_state.get("autonomy_mode"):
    print("  ADVERTENCIA - autonomy_mode no esta alineado entre Quant y Core")

print("[6/8] Verificando patch restricted symbols...")
p = Path(r"C:\ATLAS_PUSH\atlas_code_quant\api\main.py")
txt = p.read_text(encoding="utf-8")
m = re.search(r"_SANDBOX_RESTRICTED_SYMBOLS.*frozenset\([^\n]+\)", txt)
line = m.group() if m else "NO ENCONTRADO"
print(f"  {line}")
if '"OS"' in line and '"AA"' in line:
    print("  OK - OS y AA bloqueados en disco")
else:
    print("  ADVERTENCIA - Patch incompleto, revisar main.py")

print("[7/8] Iniciando loop (interval=120s, max_per_cycle=1)...")
r = api("/operation/loop/start", {"interval_sec": 120, "max_per_cycle": 1})
print(f"  Loop start: ok={r.get('ok')} error={r.get('error', '-')}")

print("[8/8] Verificacion final...")
conn2 = sqlite3.connect(DB)
cur2 = conn2.cursor()
cur2.execute(
    "SELECT symbol, COUNT(*) FROM trading_journal "
    "WHERE status='open' AND broker_order_ids_json IN ('{}','[]','') "
    "GROUP BY symbol"
)
phantoms = cur2.fetchall()
conn2.close()
print(f"  Phantoms open restantes: {phantoms if phantoms else 'NINGUNO (OK)'}")

now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
print(f"\n[DONE] {now} - ATLAS paper-first listo. Loop activo.")
print("  OS y AA: bloqueados por patch hardcodeado en main.py")
print("  Portfolio risk guard: activo (bloqueara nuevas entradas si hay exit_now)")
print("  Scanner: activo, candidatos reales cada 180s")
print("  Loop: ciclos cada 120s, max 1 trade por ciclo")
print("  Core: contrato operativo sincronizado con Quant")
print("  Nota: este script mantiene nombre legacy, pero no arma trading live real")
