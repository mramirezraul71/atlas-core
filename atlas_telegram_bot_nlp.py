"""
ATLAS Telegram Bot NLP v3.0

Bot conversacional para interactuar con ATLAS en lenguaje natural.
Lee credenciales y endpoints desde C:\\ATLAS_PUSH\\.env.
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
OLLAMA_URL = os.environ.get("ATLAS_OLLAMA_URL", "http://localhost:11434/api/generate")
JOURNAL_DB = os.environ.get(
    "ATLAS_JOURNAL_DB",
    r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3",
)
STATE_PATH = os.environ.get(
    "ATLAS_STATE_PATH",
    r"C:\ATLAS_PUSH\atlas_code_quant\data\operation\operation_center_state.json",
)
TELEGRAM_URL = f"https://api.telegram.org/bot{BOT_TOKEN}"
OLLAMA_MODELS = [
    "deepseek-coder-v2:16b",
    "qwen2.5-coder:7b",
    "llama3.1:8b",
    "llama3:8b",
    "qwen2.5:7b",
]

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(r"C:\ATLAS_PUSH\telegram_bot_nlp.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger("ATLAS-NLP")

conversation_history: list[dict[str, str]] = []
MAX_HISTORY = 12
ACTIVE_MODEL: str | None = None


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
            headers={"X-Token": MCP_TOKEN},
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


def _load_state() -> tuple[dict | None, str]:
    state_payload = mcp({"action": "get_state"})
    if not state_payload:
        return None, STATE_PATH
    return state_payload.get("state", {}), state_payload.get("path", STATE_PATH)


def _set_fail_safe(active: bool) -> bool:
    state, path = _load_state()
    if state is None:
        return False
    state["fail_safe_active"] = active
    state["fail_safe_reason"] = "Pausado manualmente via Telegram NLP" if active else None
    result = mcp(
        {
            "action": "edit_file",
            "path": path,
            "content": json.dumps(state, indent=2, ensure_ascii=False),
        }
    )
    return result is not None


def get_atlas_context() -> str:
    parts: list[str] = []
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M %Z")
    parts.append(f"Fecha/hora actual: {now}")

    state_payload = mcp({"action": "get_state"})
    state = state_payload.get("state", {}) if state_payload else None
    if state:
        fail_safe = "ACTIVO" if state.get("fail_safe_active") else "inactivo"
        guard = state.get("post_journal_rebuild_guard", {}) or {}
        guard_txt = "activo" if guard.get("active") else "inactivo"
        parts.append(
            "Estado ATLAS: "
            f"modo={state.get('auton_mode')} | "
            f"cuenta={state.get('account_scope')} | "
            f"fail_safe={fail_safe} | "
            f"errores_operacionales={state.get('operational_error_count', 0)} | "
            f"guard_rebuild={guard_txt}"
        )
    else:
        parts.append("Estado ATLAS: MCP no responde")

    trades_today = mcp(
        {
            "action": "query_db",
            "db": JOURNAL_DB,
            "sql": (
                "SELECT COUNT(*) AS total, "
                "SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) AS wins, "
                "ROUND(COALESCE(SUM(realized_pnl), 0), 2) AS pnl_total "
                "FROM trading_journal "
                "WHERE DATE(entry_time) = DATE('now','localtime') "
                "AND status = 'closed'"
            ),
        }
    )
    rows = trades_today.get("rows", []) if trades_today else []
    if rows:
        row = rows[0]
        total = int(row.get("total") or 0)
        wins = int(row.get("wins") or 0)
        pnl = float(row.get("pnl_total") or 0.0)
        win_rate = (wins / total * 100.0) if total else 0.0
        parts.append(
            f"Trades hoy (cerrados): {total} | wins={wins} | WR={win_rate:.0f}% | PnL=${pnl:.2f}"
        )

    latest_trades = mcp(
        {
            "action": "query_db",
            "db": JOURNAL_DB,
            "sql": (
                "SELECT symbol, strategy_type, entry_price, realized_pnl, status, entry_time "
                "FROM trading_journal "
                "WHERE DATE(entry_time) = DATE('now','localtime') "
                "ORDER BY entry_time DESC LIMIT 5"
            ),
        }
    )
    rows = latest_trades.get("rows", []) if latest_trades else []
    if rows:
        summary = " | ".join(
            f"{row.get('symbol')}({row.get('strategy_type')}) "
            f"PnL=${float(row.get('realized_pnl') or 0):.2f} [{row.get('status')}]"
            for row in rows
        )
        parts.append(f"Ultimos trades: {summary}")

    return "\n".join(parts)


def get_available_model() -> str | None:
    try:
        response = requests.get("http://localhost:11434/api/tags", timeout=5)
        response.raise_for_status()
        available = [model["name"] for model in response.json().get("models", [])]
        log.info("Modelos disponibles: %s", available)
        for preferred in OLLAMA_MODELS:
            for candidate in available:
                if preferred.split(":")[0] in candidate:
                    return candidate
    except Exception as exc:
        log.warning("Ollama no disponible: %s", exc)
    return None


def ask_llm(user_message: str, atlas_context: str) -> str:
    if not ACTIVE_MODEL:
        return ask_llm_fallback(user_message, atlas_context)

    system_prompt = (
        "Eres el asistente inteligente de ATLAS, un sistema de trading algoritmico en paper mode. "
        "Respondes en espanol, de forma concisa y directa como un trader profesional. "
        "Si te piden pausar o reanudar ATLAS, indicalo con claridad. "
        "Si el sistema no responde o faltan datos, dilo explicitamente.\n\n"
        f"CONTEXTO ACTUAL DEL SISTEMA:\n{atlas_context}\n\n"
        "Responde en maximo 4-6 lineas salvo que pidan un reporte detallado."
    )

    dialogue = []
    for turn in conversation_history[-MAX_HISTORY:]:
        dialogue.append(f"Usuario: {turn['user']}")
        dialogue.append(f"ATLAS: {turn['assistant']}")
    dialogue.append(f"Usuario: {user_message}")
    dialogue.append("ATLAS:")

    payload = {
        "model": ACTIVE_MODEL,
        "prompt": (
            f"<s>[INST] <<SYS>>\n{system_prompt}\n<</SYS>>\n\n"
            + "\n".join(dialogue)
            + " [/INST]"
        ),
        "stream": False,
        "options": {
            "temperature": 0.3,
            "num_predict": 300,
            "stop": ["Usuario:", "User:"],
        },
    }

    try:
        response = requests.post(OLLAMA_URL, json=payload, timeout=60)
        response.raise_for_status()
        return response.json().get("response", "").strip() or ask_llm_fallback(
            user_message, atlas_context
        )
    except Exception as exc:
        log.error("LLM error: %s", exc)
        return ask_llm_fallback(user_message, atlas_context)


def ask_llm_fallback(user_message: str, atlas_context: str) -> str:
    msg = user_message.lower()

    if any(word in msg for word in ["estado", "como esta", "como esta?", "status", "ok"]):
        lines = atlas_context.split("\n")
        return "\n".join(lines[1:3]) if len(lines) > 2 else atlas_context

    if any(word in msg for word in ["trade", "operacion", "compra", "venta", "posicion"]):
        for line in atlas_context.split("\n"):
            if "trades" in line.lower():
                return line
        return "Sin trades registrados hoy."

    if any(word in msg for word in ["pnl", "ganancia", "perdida", "dinero", "profit", "resultado"]):
        for line in atlas_context.split("\n"):
            if "pnl" in line.lower():
                return line
        return "Sin PnL registrado hoy."

    if any(word in msg for word in ["pausa", "pausar", "stop", "detener", "para"]):
        if _set_fail_safe(True):
            return "Trading pausado. Fail-safe activado. ATLAS no ejecutara nuevas ordenes."
        return "No pude conectar con el MCP para pausar."

    if any(word in msg for word in ["reanudar", "continuar", "resume", "activar", "encender"]):
        if _set_fail_safe(False):
            return "Trading reanudado. Fail-safe desactivado."
        return "No pude conectar con el MCP para reanudar."

    if any(word in msg for word in ["error", "fallo", "problema", "issue"]):
        state_payload = mcp({"action": "get_state"})
        state = state_payload.get("state", {}) if state_payload else None
        if state:
            count = state.get("operational_error_count", 0)
            last = state.get("last_operational_error") or "Ninguno"
            return f"Errores operacionales: {count}\nUltimo: {last}"

    if any(word in msg for word in ["hola", "buenas", "hey", "hi", "hello"]):
        return "Hola. ATLAS operando en paper mode. En que te ayudo?"

    if any(word in msg for word in ["ayuda", "help", "que puedes", "comandos"]):
        return (
            "Puedes preguntarme en lenguaje natural:\n"
            "- Como esta ATLAS?\n"
            "- Cuantos trades hubo hoy?\n"
            "- Cual es el PnL?\n"
            "- Pausa el trading\n"
            "- Hay errores?\n"
            "- Dame el estado completo"
        )

    return f"Entendido. Estado actual:\n{atlas_context[:400]}"


def check_and_execute_action(user_message: str, llm_response: str) -> None:
    msg = user_message.lower()

    if any(word in msg for word in ["pausa", "pausar", "detener trading", "stop trading"]):
        if _set_fail_safe(True):
            log.info("Accion ejecutada: fail_safe=True")

    elif any(word in msg for word in ["reanudar", "reiniciar trading", "resume trading"]):
        if _set_fail_safe(False):
            log.info("Accion ejecutada: fail_safe=False")


def send(text: str) -> bool:
    try:
        response = requests.post(
            f"{TELEGRAM_URL}/sendMessage",
            json={"chat_id": CHAT_ID, "text": text},
            timeout=10,
        )
        return response.ok
    except Exception as exc:
        log.error("Telegram send error: %s", exc)
        return False


def send_typing() -> None:
    try:
        requests.post(
            f"{TELEGRAM_URL}/sendChatAction",
            json={"chat_id": CHAT_ID, "action": "typing"},
            timeout=5,
        )
    except Exception:
        pass


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


def remove_commands() -> None:
    try:
        requests.post(f"{TELEGRAM_URL}/deleteMyCommands", timeout=5)
        log.info("Menu de comandos eliminado - modo conversacional activado")
    except Exception:
        pass


def main() -> None:
    global ACTIVE_MODEL

    log.info("=" * 60)
    log.info("ATLAS Telegram Bot NLP v3.0 iniciando...")

    ACTIVE_MODEL = get_available_model()
    if ACTIVE_MODEL:
        log.info("Modelo LLM activo: %s", ACTIVE_MODEL)
    else:
        log.warning("Ollama no disponible - modo fallback por keywords")

    remove_commands()

    model_info = f"Cerebro: {ACTIVE_MODEL}" if ACTIVE_MODEL else "Modo: interpretacion por keywords"
    send(
        "ATLAS listo para conversar.\n\n"
        f"{model_info}\n\n"
        "Escribeme en lenguaje natural:\n"
        '"Como esta el sistema?"\n'
        '"Hubo trades hoy?"\n'
        '"Pausa el trading"\n'
        '"Dame el PnL de hoy"'
    )

    offset = 0
    log.info("Loop conversacional iniciado.")

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
                    continue

                user_text = message["text"].strip()
                log.info("Usuario: %r", user_text)

                send_typing()
                atlas_context = get_atlas_context()
                response = ask_llm(user_text, atlas_context)
                check_and_execute_action(user_text, response)

                conversation_history.append({"user": user_text, "assistant": response})
                if len(conversation_history) > MAX_HISTORY:
                    conversation_history.pop(0)

                send(response)
                log.info("ATLAS: %r", response[:160])

        except KeyboardInterrupt:
            log.info("Bot detenido.")
            send("ATLAS Bot offline.")
            break
        except Exception as exc:
            log.error("Error en loop: %s", exc, exc_info=True)
            time.sleep(5)


if __name__ == "__main__":
    main()
