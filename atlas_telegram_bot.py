"""
ATLAS Telegram Bot Interactivo v2.0

Bot de comandos para monitorear y controlar ATLAS via Telegram.
Usa el MCP local en localhost:8799 y lee credenciales desde .env.

Nota: no ejecutar en paralelo con atlas_telegram_bot_nlp.py usando el mismo token.
"""

import datetime
import json
import logging
import os
import time
from pathlib import Path

import requests


def _load_env() -> None:
    env_path = Path(__file__).parent / ".env"
    if not env_path.exists():
        return
    for line in env_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        os.environ.setdefault(key.strip(), value.strip())


_load_env()


BOT_TOKEN = os.environ["ATLAS_BOT_TOKEN"]
CHAT_ID = os.environ["ATLAS_CHAT_ID"]
MCP_URL = os.environ.get("ATLAS_MCP_URL", "http://localhost:8799/execute")
MCP_TOKEN = os.environ.get("ATLAS_MCP_TOKEN", "atlas_mcp_2026")
JOURNAL_DB = os.environ.get(
    "ATLAS_JOURNAL_DB",
    r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3",
)
STATE_PATH = os.environ.get(
    "ATLAS_STATE_PATH",
    r"C:\ATLAS_PUSH\atlas_code_quant\data\operation\operation_center_state.json",
)
TELEGRAM_URL = f"https://api.telegram.org/bot{BOT_TOKEN}"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(r"C:\ATLAS_PUSH\telegram_bot.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger("ATLAS-BOT")


def mcp(payload: dict, timeout: int = 15) -> dict | None:
    action = payload.get("action")
    if not action:
        raise ValueError("payload missing action")

    params = payload.get("params")
    if params is None:
        params = {k: v for k, v in payload.items() if k != "action"}

    try:
        response = requests.post(
            MCP_URL,
            json={"action": action, "params": params},
            headers={"X-Token": MCP_TOKEN, "Content-Type": "application/json"},
            timeout=timeout,
        )
        response.raise_for_status()
        body = response.json()
        if not body.get("ok", False):
            log.error("MCP rejected action %s: %s", action, body)
            return None
        return body.get("data")
    except Exception as exc:
        log.error("MCP error on %s: %s", action, exc)
        return None


def query_journal(sql: str) -> dict | None:
    return mcp({"action": "query_db", "db": JOURNAL_DB, "sql": sql})


def get_state_payload() -> dict | None:
    return mcp({"action": "get_state"})


def get_state() -> dict | None:
    payload = get_state_payload()
    return payload.get("state", {}) if payload else None


def get_logs_payload(n: int = 30) -> dict | None:
    return mcp({"action": "get_logs", "n": n})


def _set_fail_safe(active: bool) -> bool:
    result = mcp(
        {
            "action": "set_state",
            "key": "fail_safe_active",
            "value": active,
            "reason": "Pausado manualmente via Telegram" if active else None,
        }
    )
    return result is not None


def send(text: str, parse_mode: str = "Markdown") -> bool:
    try:
        response = requests.post(
            f"{TELEGRAM_URL}/sendMessage",
            json={"chat_id": CHAT_ID, "text": text, "parse_mode": parse_mode},
            timeout=10,
        )
        return response.ok
    except Exception as exc:
        log.error("Telegram send error: %s", exc)
        return False


def get_updates(offset: int = 0) -> list:
    try:
        response = requests.get(
            f"{TELEGRAM_URL}/getUpdates",
            params={"offset": offset, "timeout": 30, "allowed_updates": ["message"]},
            timeout=35,
        )
        response.raise_for_status()
        return response.json().get("result", [])
    except Exception as exc:
        log.error("getUpdates error: %s", exc)
        return []


def set_commands() -> None:
    commands = [
        {"command": "estado", "description": "Estado completo de ATLAS"},
        {"command": "trades", "description": "Trades de hoy"},
        {"command": "pnl", "description": "PnL acumulado del dia"},
        {"command": "logs", "description": "Ultimos logs del sistema"},
        {"command": "journal", "description": "Estadisticas del journal (7 dias)"},
        {"command": "snapshot", "description": "Snapshot de politica adaptativa"},
        {"command": "pausa", "description": "Pausar trading (fail-safe ON)"},
        {"command": "reanudar", "description": "Reanudar trading (fail-safe OFF)"},
        {"command": "guard", "description": "Estado del post-rebuild guard"},
        {"command": "errores", "description": "Errores operacionales del dia"},
        {"command": "ayuda", "description": "Lista de comandos disponibles"},
    ]
    try:
        requests.post(f"{TELEGRAM_URL}/setMyCommands", json={"commands": commands}, timeout=10)
        log.info("Comandos registrados en Telegram")
    except Exception as exc:
        log.warning("No se pudieron registrar comandos: %s", exc)


def fmt_estado(state: dict) -> str:
    fail_safe = "ACTIVO" if state.get("fail_safe_active") else "inactivo"
    guard = state.get("post_journal_rebuild_guard", {}) or {}
    if guard.get("active"):
        expires_raw = guard.get("expires_at", "")
        try:
            expires_dt = datetime.datetime.fromisoformat(expires_raw)
            guard_txt = f"ACTIVO hasta {expires_dt.strftime('%H:%M')}"
        except Exception:
            guard_txt = f"ACTIVO ({expires_raw[:16]})"
    else:
        guard_txt = "inactivo"

    now = datetime.datetime.now().strftime("%H:%M:%S")
    return (
        f"*ATLAS Estado* - {now}\n\n"
        f"Cuenta: `{state.get('account_scope', '?')}`\n"
        f"Modo: `{state.get('auton_mode', '?')}`\n"
        f"Fail-safe: `{fail_safe}`\n"
        f"Vision: `{state.get('vision_mode', '?')}`\n"
        f"WR minimo: `{state.get('min_auton_win_rate_pct', '?')}%`\n"
        f"Errores hoy: `{state.get('operational_error_count', 0)}`\n"
        f"Guard rebuild: `{guard_txt}`\n"
    )


def cmd_estado() -> str:
    state = get_state()
    if state is None:
        return "*MCP server no responde* - verificar que atlas_mcp_server.py este corriendo."
    return fmt_estado(state)


def cmd_trades() -> str:
    sql = (
        "SELECT strategy_type, symbol, entry_price, exit_price, realized_pnl, status, entry_time "
        "FROM trading_journal "
        "WHERE DATE(entry_time) = DATE('now','localtime') "
        "ORDER BY entry_time DESC LIMIT 15"
    )
    result = query_journal(sql)
    if result is None:
        return "Error consultando journal - MCP no responde."

    rows = result.get("rows", [])
    if not rows:
        return "*Trades hoy* - Sin operaciones registradas aun."

    lines = [f"*Trades hoy* ({len(rows)} registros)\n"]
    for row in rows:
        symbol = row.get("symbol", "?")
        strategy = row.get("strategy_type", "?")
        entry_price = float(row.get("entry_price") or 0.0)
        exit_price = row.get("exit_price")
        pnl = float(row.get("realized_pnl") or 0.0)
        status = row.get("status", "?")
        entry_time = str(row.get("entry_time", ""))[:16]
        exit_text = "-" if exit_price is None else f"{float(exit_price):.2f}"
        lines.append(
            f"`{symbol}` [{strategy}] E:${entry_price:.2f} X:${exit_text} "
            f"PnL:${pnl:.2f} [{status}] {entry_time}"
        )
    return "\n".join(lines)


def cmd_pnl() -> str:
    sql = (
        "SELECT COUNT(*) AS total, "
        "SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) AS wins, "
        "SUM(CASE WHEN realized_pnl <= 0 THEN 1 ELSE 0 END) AS losses, "
        "ROUND(COALESCE(SUM(realized_pnl), 0), 2) AS total_pnl, "
        "ROUND(COALESCE(AVG(realized_pnl), 0), 2) AS avg_pnl, "
        "ROUND(COALESCE(MAX(realized_pnl), 0), 2) AS best_trade, "
        "ROUND(COALESCE(MIN(realized_pnl), 0), 2) AS worst_trade "
        "FROM trading_journal "
        "WHERE DATE(entry_time) = DATE('now','localtime') AND status = 'closed'"
    )
    result = query_journal(sql)
    if result is None:
        return "Error consultando journal."

    rows = result.get("rows", [])
    if not rows:
        return "*PnL Hoy* - Sin trades cerrados aun."

    row = rows[0]
    total = int(row.get("total") or 0)
    wins = int(row.get("wins") or 0)
    losses = int(row.get("losses") or 0)
    total_pnl = float(row.get("total_pnl") or 0.0)
    avg_pnl = float(row.get("avg_pnl") or 0.0)
    best_trade = float(row.get("best_trade") or 0.0)
    worst_trade = float(row.get("worst_trade") or 0.0)
    win_rate = (wins / total * 100.0) if total else 0.0

    return (
        "*PnL del Dia*\n\n"
        f"Trades cerrados: `{total}`\n"
        f"Ganadores: `{wins}` ({win_rate:.1f}%)\n"
        f"Perdedores: `{losses}`\n"
        f"PnL total: `${total_pnl:.2f}`\n"
        f"PnL promedio: `${avg_pnl:.2f}`\n"
        f"Mejor trade: `${best_trade:.2f}`\n"
        f"Peor trade: `${worst_trade:.2f}`\n"
    )


def cmd_logs() -> str:
    result = get_logs_payload(25)
    if result is None:
        return "MCP no responde para logs."

    lines = result.get("lines", [])
    if not lines:
        return "*Logs* - Sin entradas recientes."

    response = ["*Ultimos logs ATLAS*\n"]
    for entry in lines[-20:]:
        response.append(f"`{str(entry)[:140]}`")
    return "\n".join(response)


def cmd_journal() -> str:
    sql = (
        "SELECT DATE(entry_time) AS dia, COUNT(*) AS trades, "
        "SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) AS wins, "
        "ROUND(COALESCE(SUM(realized_pnl), 0), 2) AS pnl "
        "FROM trading_journal "
        "WHERE entry_time >= DATE('now','localtime','-7 days') "
        "GROUP BY dia ORDER BY dia DESC"
    )
    result = query_journal(sql)
    if result is None:
        return "Error consultando journal."

    rows = result.get("rows", [])
    if not rows:
        return "*Journal 7 dias* - Sin registros."

    lines = ["*Journal - ultimos 7 dias*\n"]
    for row in rows:
        day = row.get("dia", "?")
        trades = int(row.get("trades") or 0)
        wins = int(row.get("wins") or 0)
        pnl = float(row.get("pnl") or 0.0)
        win_rate = (wins / trades * 100.0) if trades else 0.0
        lines.append(f"`{day}` - {trades} trades | WR {win_rate:.0f}% | PnL ${pnl:.2f}")
    return "\n".join(lines)


def cmd_snapshot() -> str:
    result = mcp(
        {
            "action": "read_file",
            "path": r"C:\ATLAS_PUSH\atlas_code_quant\data\learning\adaptive_policy_snapshot.json",
        }
    )
    if result is None:
        return "MCP no responde."

    try:
        content = json.loads(result.get("content", "{}"))
    except Exception:
        content = {}

    return (
        "*Snapshot Politica Adaptativa*\n\n"
        f"Score threshold: `{content.get('global_score_threshold', '?')}`\n"
        f"Risk mode: `{content.get('risk_mode', content.get('mode', '?'))}`\n"
        f"Trades registrados: `{content.get('trade_count', content.get('total_trades', '?'))}`\n"
        f"Ultima actualizacion: `{str(content.get('timestamp', content.get('updated_at', '?')))[:19]}`\n"
    )


def cmd_pausa() -> str:
    if not _set_fail_safe(True):
        return "MCP no responde - pausa no ejecutada."
    return "*Trading PAUSADO* - fail_safe_active = True"


def cmd_reanudar() -> str:
    if not _set_fail_safe(False):
        return "MCP no responde - reanudacion no ejecutada."
    return "*Trading REANUDADO* - fail_safe_active = False"


def cmd_guard() -> str:
    state = get_state()
    if state is None:
        return "MCP no responde."
    guard = state.get("post_journal_rebuild_guard", {}) or {}
    if not guard.get("active"):
        return "*Guard rebuild* - INACTIVO"

    return (
        "*Guard Rebuild*\n\n"
        f"Estado: `ACTIVO`\n"
        f"Razon: `{guard.get('reason', '?')}`\n"
        f"Activado: `{str(guard.get('activated_at', ''))[:19]}`\n"
        f"Expira: `{str(guard.get('expires_at', ''))[:19]}`\n"
    )


def cmd_errores() -> str:
    state = get_state()
    if state is None:
        return "MCP no responde."
    return (
        "*Errores Operacionales*\n\n"
        f"Errores hoy ({state.get('operational_error_day', '?')}): "
        f"`{state.get('operational_error_count', 0)}` / "
        f"`{state.get('operational_error_limit', 10)}`\n"
        f"Ultimo error: `{state.get('last_operational_error') or 'Ninguno'}`\n"
    )


def cmd_ayuda() -> str:
    return (
        "*ATLAS Bot - Comandos disponibles*\n\n"
        "*/estado* - Estado completo del sistema\n"
        "*/trades* - Trades de hoy\n"
        "*/pnl* - Resumen PnL del dia\n"
        "*/logs* - Ultimos logs del sistema\n"
        "*/journal* - Rendimiento de los ultimos 7 dias\n"
        "*/snapshot* - Politica adaptativa actual\n"
        "*/guard* - Estado del post-rebuild guard\n"
        "*/errores* - Errores operacionales del dia\n"
        "*/pausa* - Activar fail-safe\n"
        "*/reanudar* - Desactivar fail-safe\n"
        "*/ayuda* - Mostrar esta ayuda"
    )


COMMANDS = {
    "/estado": cmd_estado,
    "/trades": cmd_trades,
    "/pnl": cmd_pnl,
    "/logs": cmd_logs,
    "/journal": cmd_journal,
    "/snapshot": cmd_snapshot,
    "/pausa": cmd_pausa,
    "/reanudar": cmd_reanudar,
    "/guard": cmd_guard,
    "/errores": cmd_errores,
    "/ayuda": cmd_ayuda,
    "/start": cmd_ayuda,
    "/help": cmd_ayuda,
}


def handle_message(message: dict) -> None:
    text = message.get("text", "").strip().lower().split("@")[0]
    log.info("Mensaje recibido: %r", text)

    if text in COMMANDS:
        try:
            send(COMMANDS[text]())
            log.info("Respondido: %s", text)
        except Exception as exc:
            log.error("Error ejecutando %s: %s", text, exc, exc_info=True)
            send(f"Error ejecutando `{text}`: {exc}")
        return

    if text.startswith("/"):
        send(f"Comando no reconocido: `{text}`\nUsa /ayuda para ver los disponibles.")
        return

    send(
        "Hola. Soy el bot de comandos de ATLAS.\n"
        "Usa /ayuda para ver todos los comandos disponibles.\n"
        "_Este bot no responde a lenguaje natural._"
    )


def main() -> None:
    log.info("=" * 60)
    log.info("ATLAS Telegram Bot v2.0 iniciando...")
    log.info("MCP: %s", MCP_URL)
    log.info("Chat ID: %s", CHAT_ID)

    set_commands()
    send(
        "*ATLAS Bot Interactivo v2.0 - EN LINEA*\n\n"
        "Conectado a MCP server.\n"
        "Usa /ayuda para ver todos los comandos.\n\n"
        "_Esperando ordenes..._"
    )

    offset = 0
    log.info("Loop de polling iniciado.")

    while True:
        try:
            updates = get_updates(offset)
            for update in updates:
                offset = update["update_id"] + 1
                message = update.get("message", {})
                if not message or not message.get("text"):
                    continue
                chat_id = str(message.get("chat", {}).get("id", ""))
                if chat_id != CHAT_ID:
                    log.warning("Mensaje de chat no autorizado: %s", chat_id)
                    continue
                handle_message(message)
        except KeyboardInterrupt:
            log.info("Bot detenido por el usuario.")
            send("*ATLAS Bot OFFLINE* - Detenido manualmente.")
            break
        except Exception as exc:
            log.error("Error en loop principal: %s", exc, exc_info=True)
            time.sleep(5)


if __name__ == "__main__":
    main()
