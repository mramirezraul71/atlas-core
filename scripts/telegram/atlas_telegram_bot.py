"""
ATLAS Telegram Bot Interactivo v2.0
Bot de comandos bidireccional para monitorear y controlar ATLAS via Telegram.
Requiere: MCP server corriendo en localhost:8799

Secretos via variables de entorno o archivo .env (nunca hardcoded en repo).
Copia .env.example -> .env y completa los valores.
"""

import os
import requests
import json
import time
import datetime
import logging
from pathlib import Path

# ── Cargar .env si existe ─────────────────────────────────────────────────────
def _load_env():
    env_path = Path(__file__).parent / ".env"
    if env_path.exists():
        for line in env_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, _, v = line.partition("=")
                os.environ.setdefault(k.strip(), v.strip())

_load_env()

# ── Configuración ─────────────────────────────────────────────────────────────
BOT_TOKEN  = os.environ["ATLAS_BOT_TOKEN"]
CHAT_ID    = os.environ["ATLAS_CHAT_ID"]
MCP_URL    = os.environ.get("ATLAS_MCP_URL",  "http://localhost:8799/execute")
MCP_TOKEN  = os.environ.get("ATLAS_MCP_TOKEN", "atlas_mcp_2026")  # default seguro: no es un secreto sensible
JOURNAL_DB = os.environ.get(
    "ATLAS_JOURNAL_DB",
    r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"
)

TELEGRAM_URL = f"https://api.telegram.org/bot{BOT_TOKEN}"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(r"C:\ATLAS_PUSH\telegram_bot.log", encoding="utf-8"),
        logging.StreamHandler()
    ]
)
log = logging.getLogger("ATLAS-Bot")

# ── Helpers MCP ───────────────────────────────────────────────────────────────
def mcp(payload: dict, timeout: int = 15) -> dict | None:
    try:
        r = requests.post(
            MCP_URL,
            json=payload,
            headers={"X-Token": MCP_TOKEN, "Content-Type": "application/json"},
            timeout=timeout
        )
        r.raise_for_status()
        return r.json()
    except Exception as e:
        log.error(f"MCP error: {e}")
        return None

def query_journal(sql: str) -> dict | None:
    return mcp({"action": "query_db", "db": JOURNAL_DB, "sql": sql})

def get_state() -> dict | None:
    return mcp({"action": "get_state"})

def get_logs(n: int = 30) -> dict | None:
    return mcp({"action": "get_logs", "n": n})

# ── Helpers Telegram ──────────────────────────────────────────────────────────
def send(text: str, parse_mode: str = "Markdown") -> bool:
    try:
        r = requests.post(
            f"{TELEGRAM_URL}/sendMessage",
            json={"chat_id": CHAT_ID, "text": text, "parse_mode": parse_mode},
            timeout=10
        )
        return r.ok
    except Exception as e:
        log.error(f"Telegram send error: {e}")
        return False

def get_updates(offset: int = 0) -> list:
    try:
        r = requests.get(
            f"{TELEGRAM_URL}/getUpdates",
            params={"offset": offset, "timeout": 30, "allowed_updates": ["message"]},
            timeout=35
        )
        if r.ok:
            return r.json().get("result", [])
    except Exception as e:
        log.error(f"getUpdates error: {e}")
    return []

def set_commands():
    """Registra los comandos en el menú de Telegram."""
    commands = [
        {"command": "estado",    "description": "Estado completo de ATLAS"},
        {"command": "trades",    "description": "Trades de hoy"},
        {"command": "pnl",       "description": "PnL acumulado del día"},
        {"command": "logs",      "description": "Últimos 20 logs del sistema"},
        {"command": "journal",   "description": "Estadísticas del journal (7 días)"},
        {"command": "snapshot",  "description": "Snapshot de política adaptativa"},
        {"command": "pausa",     "description": "Pausar trading (fail-safe ON)"},
        {"command": "reanudar",  "description": "Reanudar trading (fail-safe OFF)"},
        {"command": "guard",     "description": "Estado del post-rebuild guard"},
        {"command": "errores",   "description": "Errores operacionales del día"},
        {"command": "ayuda",     "description": "Lista de comandos disponibles"},
    ]
    try:
        requests.post(f"{TELEGRAM_URL}/setMyCommands", json={"commands": commands}, timeout=10)
        log.info("Comandos registrados en Telegram")
    except Exception as e:
        log.warning(f"No se pudieron registrar comandos: {e}")

# ── Formateo de estado ────────────────────────────────────────────────────────
def fmt_estado(s: dict) -> str:
    fs = "🔴 ACTIVO" if s.get("fail_safe_active") else "✅ inactivo"
    modo = s.get("auton_mode", "?")
    errores = s.get("operational_error_count", 0)
    scope = s.get("account_scope", "?")
    vision = s.get("vision_mode", "?")
    wr_min = s.get("min_auton_win_rate_pct", "?")

    guard = s.get("post_journal_rebuild_guard", {})
    if guard.get("active"):
        exp_raw = guard.get("expires_at", "")
        try:
            exp_dt = datetime.datetime.fromisoformat(exp_raw)
            exp_edt = exp_dt - datetime.timedelta(hours=4)
            guard_txt = f"⚠️ ACTIVO hasta {exp_edt.strftime('%H:%M')} EDT"
        except Exception:
            guard_txt = f"⚠️ ACTIVO (exp: {exp_raw[:16]})"
    else:
        guard_txt = "✅ inactivo"

    now = datetime.datetime.now().strftime("%H:%M:%S EDT")
    return (
        f"📡 *ATLAS Estado* — {now}\n\n"
        f"🏦 Cuenta: `{scope}`\n"
        f"⚙️ Modo: `{modo}`\n"
        f"🛡️ Fail-safe: {fs}\n"
        f"👁️ Vision: `{vision}`\n"
        f"📊 WR mínimo: `{wr_min}%`\n"
        f"❌ Errores hoy: `{errores}`\n"
        f"🔒 Guard rebuild: {guard_txt}\n"
    )

# ── Handlers de comandos ──────────────────────────────────────────────────────
def cmd_estado():
    s = get_state()
    if s is None:
        return "🔴 *MCP server no responde* — verificar que atlas_mcp_server.py esté corriendo."
    return fmt_estado(s)

def cmd_trades():
    sql = (
        "SELECT strategy_type, symbol, entry_price, exit_price, realized_pnl, status, entry_time "
        "FROM trading_journal WHERE DATE(entry_time) = DATE('now','localtime') ORDER BY entry_time DESC LIMIT 15"
    )
    res = query_journal(sql)
    if res is None:
        return "❌ Error consultando journal — MCP no responde."

    rows = res if isinstance(res, list) else res.get("rows", res.get("data", []))
    if not rows:
        return f"📋 *Trades hoy* — Sin operaciones registradas aún."

    lines = [f"📋 *Trades hoy* ({len(rows)} registros)\n"]
    for r in rows:
        sym   = r.get("symbol", "?")
        strat = r.get("strategy_type", "?")
        ep    = r.get("entry_price", 0) or 0
        xp    = r.get("exit_price") or "—"
        pnl   = r.get("realized_pnl", 0) or 0
        st    = r.get("status", "?")
        et    = str(r.get("entry_time", ""))[:16]
        pnl_icon = "🟢" if float(pnl) >= 0 else "🔴"
        lines.append(
            f"{pnl_icon} `{sym}` [{strat}] "
            f"E:${ep:.2f} X:${xp if xp=='—' else f'{float(xp):.2f}'} "
            f"PnL:${float(pnl):.2f} [{st}] {et}"
        )
    return "\n".join(lines)

def cmd_pnl():
    sql = (
        "SELECT COUNT(*) as total, "
        "SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) as wins, "
        "SUM(CASE WHEN realized_pnl <= 0 THEN 1 ELSE 0 END) as losses, "
        "ROUND(SUM(realized_pnl),2) as total_pnl, "
        "ROUND(AVG(realized_pnl),2) as avg_pnl, "
        "ROUND(MAX(realized_pnl),2) as best_trade, "
        "ROUND(MIN(realized_pnl),2) as worst_trade "
        "FROM trading_journal WHERE DATE(entry_time) = DATE('now','localtime') AND status='closed'"
    )
    res = query_journal(sql)
    if res is None:
        return "❌ Error consultando journal."

    rows = res if isinstance(res, list) else res.get("rows", res.get("data", []))
    if not rows:
        return "📊 *PnL Hoy* — Sin trades cerrados aún."

    d = rows[0]
    total = d.get("total", 0) or 0
    wins  = d.get("wins", 0) or 0
    losses= d.get("losses", 0) or 0
    tpnl  = d.get("total_pnl", 0) or 0
    apnl  = d.get("avg_pnl", 0) or 0
    best  = d.get("best_trade", 0) or 0
    worst = d.get("worst_trade", 0) or 0
    wr    = (wins/total*100) if total > 0 else 0
    pnl_icon = "🟢" if float(tpnl) >= 0 else "🔴"

    return (
        f"📊 *PnL del Día*\n\n"
        f"Trades cerrados: `{total}`\n"
        f"Ganadores: `{wins}` ({wr:.1f}%)\n"
        f"Perdedores: `{losses}`\n"
        f"{pnl_icon} PnL total: `${float(tpnl):.2f}`\n"
        f"PnL promedio: `${float(apnl):.2f}`\n"
        f"Mejor trade: `${float(best):.2f}`\n"
        f"Peor trade: `${float(worst):.2f}`\n"
    )

def cmd_logs():
    res = get_logs(25)
    if res is None:
        return "❌ MCP no responde para logs."

    log_lines = res if isinstance(res, list) else res.get("logs", res.get("data", []))
    if not log_lines:
        return "📜 *Logs* — Sin entradas recientes."

    lines = ["📜 *Últimos logs ATLAS*\n"]
    for entry in log_lines[-20:]:
        if isinstance(entry, dict):
            ts  = str(entry.get("timestamp", entry.get("time", "")))[:19]
            lvl = entry.get("level", entry.get("levelname", "INFO"))
            msg = entry.get("message", entry.get("msg", str(entry)))[:120]
        else:
            ts, lvl, msg = "", "LOG", str(entry)[:120]
        icon = "🔴" if lvl in ("ERROR", "CRITICAL") else "🟡" if lvl == "WARNING" else "⚪"
        lines.append(f"{icon} `{ts}` {msg}")

    return "\n".join(lines)

def cmd_journal():
    sql = (
        "SELECT DATE(entry_time) as dia, COUNT(*) as trades, "
        "SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) as wins, "
        "ROUND(SUM(realized_pnl),2) as pnl "
        "FROM trading_journal WHERE entry_time >= date('now','localtime','-7 days') "
        "GROUP BY dia ORDER BY dia DESC"
    )
    res = query_journal(sql)
    if res is None:
        return "❌ Error consultando journal."

    rows = res if isinstance(res, list) else res.get("rows", res.get("data", []))
    if not rows:
        return "📅 *Journal 7 días* — Sin registros."

    lines = ["📅 *Journal — últimos 7 días*\n"]
    for r in rows:
        dia    = r.get("dia", "?")
        trades = r.get("trades", 0)
        wins   = r.get("wins", 0)
        pnl    = r.get("pnl", 0) or 0
        wr     = (wins/trades*100) if trades > 0 else 0
        icon   = "🟢" if float(pnl) >= 0 else "🔴"
        lines.append(f"{icon} `{dia}` — {trades} trades | WR {wr:.0f}% | PnL ${float(pnl):.2f}")

    return "\n".join(lines)

def cmd_snapshot():
    res = mcp({"action": "read_file",
               "path": r"C:\ATLAS_PUSH\atlas_code_quant\data\learning\adaptive_policy_snapshot.json"})
    if res is None:
        return "❌ MCP no responde."

    content = res if isinstance(res, dict) else {}
    if "content" in content:
        try:
            content = json.loads(content["content"])
        except Exception:
            pass

    score  = content.get("global_score_threshold", "?")
    mode   = content.get("risk_mode", content.get("mode", "?"))
    ts     = str(content.get("timestamp", content.get("updated_at", "?")))[:19]
    trades = content.get("trade_count", content.get("total_trades", "?"))

    return (
        f"🧠 *Snapshot Política Adaptativa*\n\n"
        f"Score threshold: `{score}`\n"
        f"Risk mode: `{mode}`\n"
        f"Trades registrados: `{trades}`\n"
        f"Última actualización: `{ts}`\n"
    )

def cmd_pausa():
    res = mcp({"action": "set_state", "key": "fail_safe_active", "value": True})
    if res is None:
        return "❌ MCP no responde — pausa NO ejecutada."
    return "🛑 *Trading PAUSADO* — fail_safe_active = True\nATLAS no ejecutará nuevas órdenes hasta /reanudar"

def cmd_reanudar():
    res = mcp({"action": "set_state", "key": "fail_safe_active", "value": False})
    if res is None:
        return "❌ MCP no responde — reanudación NO ejecutada."
    return "▶️ *Trading REANUDADO* — fail_safe_active = False\nATLAS puede ejecutar órdenes nuevamente."

def cmd_guard():
    s = get_state()
    if s is None:
        return "❌ MCP no responde."
    guard = s.get("post_journal_rebuild_guard", {})
    if not guard.get("active"):
        return "✅ *Guard rebuild* — INACTIVO\nATLAS puede operar libremente."

    activated = str(guard.get("activated_at", ""))[:19]
    expires   = str(guard.get("expires_at", ""))[:19]
    reason    = guard.get("reason", "?")
    try:
        exp_dt  = datetime.datetime.fromisoformat(guard.get("expires_at", ""))
        exp_edt = exp_dt - datetime.timedelta(hours=4)
        exp_str = exp_edt.strftime("%H:%M EDT")
        remaining = exp_dt - datetime.datetime.utcnow()
        mins = int(remaining.total_seconds() / 60)
        rem_str = f"{mins} minutos restantes" if mins > 0 else "EXPIRADO"
    except Exception:
        exp_str = expires
        rem_str = "?"

    return (
        f"🔒 *Guard Rebuild*\n\n"
        f"Estado: ⚠️ ACTIVO\n"
        f"Razón: `{reason}`\n"
        f"Activado: `{activated}`\n"
        f"Expira: `{exp_str}`\n"
        f"Restante: `{rem_str}`\n"
    )

def cmd_errores():
    s = get_state()
    if s is None:
        return "❌ MCP no responde."

    count  = s.get("operational_error_count", 0)
    day    = s.get("operational_error_day", "?")
    last_e = s.get("last_operational_error") or "Ninguno"
    limit  = s.get("operational_error_limit", 10)

    icon = "🔴" if count > 0 else "✅"
    return (
        f"❌ *Errores Operacionales*\n\n"
        f"{icon} Errores hoy ({day}): `{count}` / límite `{limit}`\n"
        f"Último error: `{last_e}`\n"
        f"{'⚠️ Auto-pausa activada' if count >= limit else '✅ Dentro de límites'}\n"
    )

def cmd_ayuda():
    return (
        "🤖 *ATLAS Bot — Comandos disponibles*\n\n"
        "*/estado* — Estado completo del sistema\n"
        "*/trades* — Trades de hoy (últimos 15)\n"
        "*/pnl* — Resumen PnL del día\n"
        "*/logs* — Últimos logs del sistema\n"
        "*/journal* — Rendimiento de los últimos 7 días\n"
        "*/snapshot* — Política adaptativa actual\n"
        "*/guard* — Estado del post-rebuild guard\n"
        "*/errores* — Errores operacionales del día\n"
        "*/pausa* — Activar fail-safe (para trading)\n"
        "*/reanudar* — Desactivar fail-safe\n"
        "*/ayuda* — Mostrar esta ayuda\n\n"
        "_ATLAS v0.6.0 — Paper Trading Mode_"
    )

# ── Router principal ──────────────────────────────────────────────────────────
COMMANDS = {
    "/estado":   cmd_estado,
    "/trades":   cmd_trades,
    "/pnl":      cmd_pnl,
    "/logs":     cmd_logs,
    "/journal":  cmd_journal,
    "/snapshot": cmd_snapshot,
    "/pausa":    cmd_pausa,
    "/reanudar": cmd_reanudar,
    "/guard":    cmd_guard,
    "/errores":  cmd_errores,
    "/ayuda":    cmd_ayuda,
    "/start":    cmd_ayuda,
    "/help":     cmd_ayuda,
}

def handle_message(msg: dict):
    text = msg.get("text", "").strip().lower().split("@")[0]  # ignora @bot_name
    log.info(f"Mensaje recibido: '{text}'")

    if text in COMMANDS:
        try:
            response = COMMANDS[text]()
            send(response)
            log.info(f"Respondido: {text}")
        except Exception as e:
            log.error(f"Error ejecutando {text}: {e}")
            send(f"❌ Error ejecutando `{text}`: {e}")
    elif text.startswith("/"):
        send(f"❓ Comando no reconocido: `{text}`\nUsa /ayuda para ver los disponibles.")
    else:
        # Mensaje libre — responder con ayuda contextual
        send(
            f"💬 Hola. Soy el bot de ATLAS.\n"
            f"Usa /ayuda para ver todos los comandos disponibles.\n"
            f"_Recibí tu mensaje, pero solo respondo a comandos._"
        )

# ── Loop principal ────────────────────────────────────────────────────────────
def main():
    log.info("=" * 60)
    log.info("ATLAS Telegram Bot v2.0 iniciando...")
    log.info("MCP: localhost:8799")
    log.info("Chat ID: " + CHAT_ID)

    # Registrar comandos en menú Telegram
    set_commands()

    # Mensaje de inicio
    send(
        "🤖 *ATLAS Bot Interactivo v2.0 — EN LÍNEA*\n\n"
        "Conectado a MCP server (localhost:8799)\n"
        "Usa /ayuda para ver todos los comandos.\n\n"
        "_Esperando órdenes..._"
    )

    offset = 0
    log.info("Loop de polling iniciado.")

    while True:
        try:
            updates = get_updates(offset)
            for update in updates:
                offset = update["update_id"] + 1
                msg = update.get("message", {})
                if msg and msg.get("text"):
                    # Solo responder al chat autorizado
                    cid = str(msg.get("chat", {}).get("id", ""))
                    if cid == CHAT_ID:
                        handle_message(msg)
                    else:
                        log.warning(f"Mensaje de chat no autorizado: {cid}")
        except KeyboardInterrupt:
            log.info("Bot detenido por el usuario.")
            send("🔴 *ATLAS Bot OFFLINE* — Detenido manualmente.")
            break
        except Exception as e:
            log.error(f"Error en loop principal: {e}")
            time.sleep(5)

if __name__ == "__main__":
    main()
