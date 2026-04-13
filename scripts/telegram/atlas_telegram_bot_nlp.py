"""
ATLAS Telegram Bot NLP v3.0
Interacción en lenguaje natural usando Ollama (DeepSeek Coder V2 / Qwen2.5)
como cerebro interpretador. Sin comandos — habla como con una persona.

Requiere:
  - MCP server corriendo en localhost:8799
  - Ollama corriendo en localhost:11434 con DeepSeek o Qwen2.5
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
BOT_TOKEN   = os.environ["ATLAS_BOT_TOKEN"]
CHAT_ID     = os.environ["ATLAS_CHAT_ID"]
MCP_URL     = os.environ.get("ATLAS_MCP_URL",   "http://localhost:8799/execute")
MCP_TOKEN   = os.environ.get("ATLAS_MCP_TOKEN",  "atlas_mcp_2026")
OLLAMA_URL  = os.environ.get("ATLAS_OLLAMA_URL", "http://localhost:11434/api/generate")
JOURNAL_DB  = os.environ.get(
    "ATLAS_JOURNAL_DB",
    r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"
)
TELEGRAM_URL = f"https://api.telegram.org/bot{BOT_TOKEN}"

# Modelo preferido — fallback automático si no está disponible
OLLAMA_MODELS = ["deepseek-coder-v2:16b", "qwen2.5-coder:7b", "llama3:8b", "qwen2.5:7b"]

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(r"C:\ATLAS_PUSH\telegram_bot_nlp.log", encoding="utf-8"),
        logging.StreamHandler()
    ]
)
log = logging.getLogger("ATLAS-NLP")

# ── Historial de conversación (contexto) ──────────────────────────────────────
conversation_history = []
MAX_HISTORY = 12  # turnos a recordar

# ── MCP helpers ───────────────────────────────────────────────────────────────
def mcp(payload: dict, timeout: int = 15) -> dict | None:
    try:
        r = requests.post(MCP_URL, json=payload,
                          headers={"X-Token": MCP_TOKEN}, timeout=timeout)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        log.error(f"MCP error: {e}")
        return None

def get_atlas_context() -> str:
    """Recopila estado actual de ATLAS para dárselo al LLM como contexto."""
    parts = []
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M EDT")
    parts.append(f"Fecha/hora actual: {now}")

    # Estado del sistema
    state = mcp({"action": "get_state"})
    if state:
        fs = "ACTIVO ⚠️" if state.get("fail_safe_active") else "inactivo"
        guard = state.get("post_journal_rebuild_guard", {})
        guard_txt = "activo" if guard.get("active") else "inactivo"
        parts.append(
            f"Estado ATLAS: modo={state.get('auton_mode')} | "
            f"cuenta={state.get('account_scope')} | "
            f"fail_safe={fs} | "
            f"errores_operacionales={state.get('operational_error_count', 0)} | "
            f"guard_rebuild={guard_txt}"
        )
    else:
        parts.append("Estado ATLAS: MCP no responde")

    # Trades de hoy
    res = mcp({
        "action": "query_db", "db": JOURNAL_DB,
        "sql": (
            "SELECT COUNT(*) as total, "
            "SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) as wins, "
            "ROUND(SUM(realized_pnl),2) as pnl_total "
            "FROM trading_journal "
            "WHERE DATE(entry_time) = DATE('now','localtime') AND status='closed'"
        )
    })
    if res:
        rows = res if isinstance(res, list) else res.get("rows", [])
        if rows:
            r = rows[0]
            total = r.get("total", 0) or 0
            wins  = r.get("wins", 0) or 0
            pnl   = r.get("pnl_total", 0) or 0
            wr    = (wins/total*100) if total > 0 else 0
            parts.append(
                f"Trades hoy (cerrados): {total} | wins={wins} | "
                f"WR={wr:.0f}% | PnL=${float(pnl):.2f}"
            )

    # Últimos 5 trades
    res2 = mcp({
        "action": "query_db", "db": JOURNAL_DB,
        "sql": (
            "SELECT symbol, strategy_type, entry_price, realized_pnl, status, entry_time "
            "FROM trading_journal "
            "WHERE DATE(entry_time) = DATE('now','localtime') "
            "ORDER BY entry_time DESC LIMIT 5"
        )
    })
    if res2:
        rows2 = res2 if isinstance(res2, list) else res2.get("rows", [])
        if rows2:
            trades_str = " | ".join(
                f"{r.get('symbol')}({r.get('strategy_type')}) "
                f"PnL=${float(r.get('realized_pnl') or 0):.2f} [{r.get('status')}]"
                for r in rows2
            )
            parts.append(f"Últimos trades: {trades_str}")

    return "\n".join(parts)

# ── Ollama helpers ────────────────────────────────────────────────────────────
def get_available_model() -> str | None:
    """Detecta qué modelo de Ollama está disponible."""
    try:
        r = requests.get("http://localhost:11434/api/tags", timeout=5)
        if r.ok:
            available = [m["name"] for m in r.json().get("models", [])]
            log.info(f"Modelos disponibles: {available}")
            for preferred in OLLAMA_MODELS:
                for avail in available:
                    if preferred.split(":")[0] in avail:
                        return avail
    except Exception as e:
        log.warning(f"Ollama no disponible: {e}")
    return None

ACTIVE_MODEL = None  # se detecta al arrancar

def ask_llm(user_message: str, atlas_context: str) -> str:
    """Envía el mensaje al LLM local con contexto de ATLAS."""
    global ACTIVE_MODEL

    if not ACTIVE_MODEL:
        return ask_llm_fallback(user_message, atlas_context)

    system_prompt = f"""Eres el asistente inteligente de ATLAS, un sistema de trading algorítmico en paper mode.
Respondes en español, de forma concisa y directa como un trader profesional.
No uses listas largas innecesariamente. Si te preguntan datos, da los números exactos.
Si te piden pausar o reanudar ATLAS, confirma que lo harás y ejecuta la acción.
Si no sabes algo o el sistema no responde, dilo claramente.

CONTEXTO ACTUAL DEL SISTEMA:
{atlas_context}

Responde siempre en máximo 4-6 líneas a menos que el usuario pida un reporte detallado."""

    # Construir historial
    messages_text = ""
    for turn in conversation_history[-MAX_HISTORY:]:
        messages_text += f"Usuario: {turn['user']}\nATLAS: {turn['assistant']}\n"
    messages_text += f"Usuario: {user_message}\nATLAS:"

    payload = {
        "model": ACTIVE_MODEL,
        "prompt": f"<s>[INST] <<SYS>>\n{system_prompt}\n<</SYS>>\n\n{messages_text} [/INST]",
        "stream": False,
        "options": {"temperature": 0.3, "num_predict": 300, "stop": ["Usuario:", "User:"]}
    }

    try:
        r = requests.post(OLLAMA_URL, json=payload, timeout=60)
        if r.ok:
            return r.json().get("response", "").strip()
    except Exception as e:
        log.error(f"LLM error: {e}")

    return ask_llm_fallback(user_message, atlas_context)

def ask_llm_fallback(user_message: str, atlas_context: str) -> str:
    """Fallback inteligente basado en keywords si Ollama no responde."""
    msg = user_message.lower()

    # Intenciones detectadas
    if any(w in msg for w in ["estado", "cómo está", "como esta", "status", "funcionando", "ok"]):
        lines = atlas_context.split("\n")
        return "\n".join(lines[1:3]) if len(lines) > 2 else atlas_context

    if any(w in msg for w in ["trade", "operacion", "operación", "compra", "venta", "posicion"]):
        for line in atlas_context.split("\n"):
            if "trades" in line.lower():
                return line
        return "Sin trades registrados hoy."

    if any(w in msg for w in ["pnl", "ganancia", "perdida", "pérdida", "dinero", "profit", "resultado"]):
        for line in atlas_context.split("\n"):
            if "pnl" in line.lower():
                return line
        return "Sin PnL registrado hoy."

    if any(w in msg for w in ["pausa", "pausar", "stop", "detener", "para"]):
        result = mcp({"action": "set_state", "key": "fail_safe_active", "value": True})
        if result is not None:
            return "Trading pausado. Fail-safe activado — ATLAS no ejecutará nuevas órdenes."
        return "No pude conectar con el MCP para pausar."

    if any(w in msg for w in ["reanudar", "continuar", "resume", "activar", "encender"]):
        result = mcp({"action": "set_state", "key": "fail_safe_active", "value": False})
        if result is not None:
            return "Trading reanudado. Fail-safe desactivado."
        return "No pude conectar con el MCP para reanudar."

    if any(w in msg for w in ["error", "fallo", "problema", "issue"]):
        state = mcp({"action": "get_state"})
        if state:
            n = state.get("operational_error_count", 0)
            last = state.get("last_operational_error") or "Ninguno"
            return f"Errores operacionales: {n}\nÚltimo: {last}"

    if any(w in msg for w in ["hola", "buenas", "hey", "hi", "hello"]):
        return "Hola. ATLAS operando en paper mode. ¿En qué te ayudo?"

    if any(w in msg for w in ["ayuda", "help", "que puedes", "qué puedes", "comandos"]):
        return (
            "Puedes preguntarme en lenguaje natural:\n"
            "— ¿Cómo está ATLAS?\n"
            "— ¿Cuántos trades hubo hoy?\n"
            "— ¿Cuál es el PnL?\n"
            "— Pausa el trading\n"
            "— ¿Hay errores?\n"
            "— Dame el estado completo"
        )

    # Respuesta genérica con contexto
    return f"Entendido. Estado actual:\n{atlas_context[:400]}"

# ── Acciones ejecutables desde el chat ───────────────────────────────────────
def check_and_execute_action(user_message: str, llm_response: str):
    """Detecta si el usuario pidió una acción y la ejecuta."""
    msg = user_message.lower()

    if any(w in msg for w in ["pausa", "pausar", "detener trading", "stop trading"]):
        mcp({"action": "set_state", "key": "fail_safe_active", "value": True})
        log.info("Acción ejecutada: fail_safe=True")

    elif any(w in msg for w in ["reanudar", "reiniciar trading", "resume trading"]):
        mcp({"action": "set_state", "key": "fail_safe_active", "value": False})
        log.info("Acción ejecutada: fail_safe=False")

# ── Telegram helpers ──────────────────────────────────────────────────────────
def send(text: str) -> bool:
    try:
        r = requests.post(
            f"{TELEGRAM_URL}/sendMessage",
            json={"chat_id": CHAT_ID, "text": text},
            timeout=10
        )
        return r.ok
    except Exception as e:
        log.error(f"Telegram send error: {e}")
        return False

def send_typing():
    try:
        requests.post(
            f"{TELEGRAM_URL}/sendChatAction",
            json={"chat_id": CHAT_ID, "action": "typing"},
            timeout=5
        )
    except Exception:
        pass

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

def remove_commands():
    """Elimina el menú de comandos para modo conversacional puro."""
    try:
        requests.post(f"{TELEGRAM_URL}/deleteMyCommands", timeout=5)
        log.info("Menú de comandos eliminado — modo conversacional activado")
    except Exception:
        pass

# ── Loop principal ─────────────────────────────────────────────────────────────
def main():
    global ACTIVE_MODEL

    log.info("=" * 60)
    log.info("ATLAS Telegram Bot NLP v3.0 iniciando...")

    # Detectar modelo disponible
    ACTIVE_MODEL = get_available_model()
    if ACTIVE_MODEL:
        log.info(f"Modelo LLM activo: {ACTIVE_MODEL}")
    else:
        log.warning("Ollama no disponible — modo fallback por keywords")

    # Eliminar menú de comandos → modo conversacional
    remove_commands()

    model_info = f"Cerebro: {ACTIVE_MODEL}" if ACTIVE_MODEL else "Modo: interpretación por keywords"

    send(
        "ATLAS listo para conversar.\n\n"
        f"{model_info}\n\n"
        "Escríbeme en lenguaje natural:\n"
        "\"¿Cómo está el sistema?\"\n"
        "\"¿Hubo trades hoy?\"\n"
        "\"Pausa el trading\"\n"
        "\"Dame el PnL de hoy\""
    )

    offset = 0
    log.info("Loop conversacional iniciado.")

    while True:
        try:
            updates = get_updates(offset)
            for update in updates:
                offset = update["update_id"] + 1
                msg = update.get("message", {})
                if not msg or not msg.get("text"):
                    continue

                cid = str(msg.get("chat", {}).get("id", ""))
                if cid != CHAT_ID:
                    continue

                user_text = msg["text"].strip()
                log.info(f"Usuario: '{user_text}'")

                # Mostrar "escribiendo..."
                send_typing()

                # Recopilar contexto actual de ATLAS
                atlas_ctx = get_atlas_context()

                # Consultar LLM
                response = ask_llm(user_text, atlas_ctx)

                # Ejecutar acción si corresponde
                check_and_execute_action(user_text, response)

                # Guardar en historial
                conversation_history.append({
                    "user": user_text,
                    "assistant": response
                })
                if len(conversation_history) > MAX_HISTORY:
                    conversation_history.pop(0)

                # Responder
                send(response)
                log.info(f"ATLAS: '{response[:80]}...'")

        except KeyboardInterrupt:
            log.info("Bot detenido.")
            send("ATLAS Bot offline.")
            break
        except Exception as e:
            log.error(f"Error en loop: {e}")
            time.sleep(5)


if __name__ == "__main__":
    main()
