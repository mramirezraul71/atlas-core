"""
ATLAS Telegram Bot NLP v3.0
Interacción en lenguaje natural usando Ollama (DeepSeek Coder V2 / Qwen2.5)
como cerebro interpretador. Sin comandos — habla como con una persona.

Requiere:
  - MCP server corriendo en localhost:8799
  - Ollama corriendo en localhost:11434 con DeepSeek o Qwen2.5
"""

import atexit
import json
import logging
import os
import re
import signal
import sys
import time
import datetime
from collections import deque
from pathlib import Path

import requests

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

# ── Configuración (BOT_TOKEN / ATLAS_BOT_TOKEN, etc.) ─────────────────────────
def _env_required(*keys: str) -> str:
    for k in keys:
        v = os.environ.get(k)
        if v:
            return v.strip()
    raise RuntimeError(f"Falta variable en .env: {' o '.join(keys)}")


BOT_TOKEN = _env_required("BOT_TOKEN", "ATLAS_BOT_TOKEN")
CHAT_ID = str(_env_required("CHAT_ID", "ATLAS_CHAT_ID"))
MCP_URL = os.environ.get("ATLAS_MCP_URL", "http://localhost:8799/execute")
MCP_TOKEN = os.environ.get("MCP_TOKEN") or os.environ.get("ATLAS_MCP_TOKEN") or "atlas_mcp_2026"
OLLAMA_URL = os.environ.get("ATLAS_OLLAMA_URL", "http://localhost:11434/api/generate")
JOURNAL_DB = os.environ.get(
    "ATLAS_JOURNAL_DB",
    r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3",
)
ATLAS_ROOT = Path(r"C:\ATLAS_PUSH").resolve()
LOCK_PATH = Path(r"C:\ATLAS_PUSH\bot.lock")
# Mismo path que atlas_mcp_server.LOG_PATH — si no existe, get_logs devuelve 500 en el MCP
QUANT_BRIDGE_LOG = Path(r"C:\ATLAS_PUSH\atlas_code_quant\logs\quant_brain_bridge.jsonl")
TELEGRAM_URL = f"https://api.telegram.org/bot{BOT_TOKEN}"

# Modelo preferido — fallback automático si no está disponible
OLLAMA_MODELS = ["deepseek-coder-v2:16b", "qwen2.5-coder:7b", "llama3:8b", "qwen2.5:7b"]

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(r"C:\ATLAS_PUSH\telegram_bot_nlp.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger("ATLAS-NLP")

# ── Historial de conversación (contexto) ──────────────────────────────────────
conversation_history = []
MAX_HISTORY = 12  # turnos a recordar

# ── Anti-duplicados (mismo update_id no debe enviar dos respuestas) ─────────
_SEEN_UPDATE_ORDER: deque[int] = deque(maxlen=500)
_SEEN_UPDATE_IDS: set[int] = set()


def should_process_update(update_id: int) -> bool:
    if update_id in _SEEN_UPDATE_IDS:
        return False
    if len(_SEEN_UPDATE_ORDER) == _SEEN_UPDATE_ORDER.maxlen:
        drop = _SEEN_UPDATE_ORDER.popleft()
        _SEEN_UPDATE_IDS.discard(drop)
    _SEEN_UPDATE_ORDER.append(update_id)
    _SEEN_UPDATE_IDS.add(update_id)
    return True


# ── Lock de instancia única ─────────────────────────────────────────────────
def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    if sys.platform == "win32":
        import ctypes

        PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
        h = ctypes.windll.kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid)
        if not h:
            return False
        ctypes.windll.kernel32.CloseHandle(h)
        return True
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _release_bot_lock():
    try:
        if LOCK_PATH.exists():
            LOCK_PATH.unlink()
    except OSError as e:
        log.warning(f"No pude borrar bot.lock: {e}")


def acquire_bot_lock() -> None:
    """
    Una sola instancia: creación atómica de bot.lock (evita dos pythonw en la misma máquina).
    """
    pid_bytes = str(os.getpid()).encode("utf-8")
    while True:
        try:
            fd = os.open(str(LOCK_PATH), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            try:
                os.write(fd, pid_bytes)
            finally:
                os.close(fd)
            break
        except FileExistsError:
            try:
                old_pid = int(LOCK_PATH.read_text(encoding="utf-8").strip())
            except (ValueError, OSError):
                LOCK_PATH.unlink(missing_ok=True)
                continue
            if _pid_alive(old_pid):
                log.error(f"Bot ya está corriendo (PID={old_pid})")
                sys.exit(1)
            try:
                LOCK_PATH.unlink()
            except OSError:
                time.sleep(0.05)

    atexit.register(_release_bot_lock)

    def _on_signal(signum, frame):
        _release_bot_lock()
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _on_signal)


# ── MCP helpers ───────────────────────────────────────────────────────────────
def mcp(payload: dict, timeout: int = 15) -> dict | list | None:
    """
    POST al MCP. Devuelve el campo 'data' de la respuesta si ok=True;
    si no, None. El servidor atlas_mcp_server usa siempre payload['params']
    para argumentos (query_db, get_logs, read_file, etc.).
    """
    try:
        r = requests.post(
            MCP_URL, json=payload, headers={"X-Token": MCP_TOKEN}, timeout=timeout
        )
        r.raise_for_status()
        body = r.json()
        if not body.get("ok"):
            log.error(f"MCP ok=false: {body}")
            return None
        return body.get("data")
    except Exception as e:
        log.error(f"MCP error: {e}")
        return None


def get_atlas_context() -> tuple[str, dict]:
    """Recopila estado actual de ATLAS. Retorna (texto_contexto, meta)."""
    meta: dict = {"state_ok": False}
    parts = []
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    parts.append(f"Fecha/hora actual: {now}")

    state_wrap = mcp({"action": "get_state"})
    state: dict = {}
    if state_wrap and isinstance(state_wrap, dict):
        state = state_wrap.get("state") or {}
    if isinstance(state, dict) and state:
        meta["state_ok"] = True
        fs = "ACTIVO ⚠️" if state.get("fail_safe_active") else "inactivo"
        guard = state.get("post_journal_rebuild_guard") or {}
        guard_txt = "activo" if guard.get("active") else "inactivo"
        parts.append(
            f"Estado ATLAS: modo={state.get('auton_mode')} | "
            f"cuenta={state.get('account_scope')} | "
            f"fail_safe={fs} | "
            f"errores_operacionales={state.get('operational_error_count', 0)} | "
            f"guard_rebuild={guard_txt}"
        )
    else:
        parts.append("Estado ATLAS: MCP no responde o sin datos de estado")

    res = mcp(
        {
            "action": "query_db",
            "params": {
                "db": JOURNAL_DB,
                "sql": (
                    "SELECT COUNT(*) as total, "
                    "SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) as wins, "
                    "ROUND(SUM(realized_pnl),2) as pnl_total "
                    "FROM trading_journal "
                    "WHERE DATE(entry_time) = DATE('now','localtime') AND status='closed'"
                ),
            },
        }
    )
    if res and isinstance(res, dict):
        rows = res.get("rows") or []
        if rows:
            r0 = rows[0]
            total = r0.get("total", 0) or 0
            wins = r0.get("wins", 0) or 0
            pnl = r0.get("pnl_total", 0) or 0
            wr = (wins / total * 100) if total > 0 else 0
            parts.append(
                f"Trades hoy (cerrados): {total} | wins={wins} | "
                f"WR={wr:.0f}% | PnL=${float(pnl):.2f}"
            )

    res2 = mcp(
        {
            "action": "query_db",
            "params": {
                "db": JOURNAL_DB,
                "sql": (
                    "SELECT symbol, strategy_type, entry_price, realized_pnl, status, entry_time "
                    "FROM trading_journal "
                    "WHERE DATE(entry_time) = DATE('now','localtime') "
                    "ORDER BY entry_time DESC LIMIT 5"
                ),
            },
        }
    )
    if res2 and isinstance(res2, dict):
        rows2 = res2.get("rows") or []
        if rows2:
            trades_str = " | ".join(
                f"{r.get('symbol')}({r.get('strategy_type')}) "
                f"PnL=${float(r.get('realized_pnl') or 0):.2f} [{r.get('status')}]"
                for r in rows2
            )
            parts.append(f"Últimos trades: {trades_str}")

    log_res = None
    if QUANT_BRIDGE_LOG.is_file():
        log_res = mcp({"action": "get_logs", "params": {"n": 8}})
    if log_res and isinstance(log_res, dict):
        raw_lines = log_res.get("lines") or []
        log_lines = []
        for entry in raw_lines[-5:]:
            if isinstance(entry, str):
                try:
                    obj = json.loads(entry)
                    ts = str(obj.get("timestamp", ""))[:19]
                    msg = str(obj.get("message", obj))[:100]
                    log_lines.append(f"{ts} {msg}")
                except json.JSONDecodeError:
                    log_lines.append(entry[:100])
            elif isinstance(entry, dict):
                ts = str(entry.get("timestamp", entry.get("time", "")))[:19]
                msg = entry.get("message", entry.get("msg", str(entry)))[:100]
                log_lines.append(f"{ts} {msg}")
            else:
                log_lines.append(str(entry)[:100])
        if log_lines:
            parts.append("Últimos logs: " + " | ".join(log_lines))

    return "\n".join(parts), meta


# ── Lectura de archivos bajo ATLAS_PUSH (MCP read_file) ──────────────────────
READ_TRIGGERS = (
    "lee ",
    "leer ",
    "léeme",
    "leeme",
    "muestra",
    "muéstrame",
    "muestrame",
    "archivo",
    "contenido",
    "abre ",
    "abrir ",
    "módulo",
    "modulo",
    "file ",
    "path ",
    "revisa ",
    "revisar ",
    "enséñame",
    "ensename",
    "codigo de",
    "código de",
    "dame el codigo",
    "dame el código",
)


def _safe_under_atlas(path_str: str) -> Path | None:
    try:
        p = Path(path_str).resolve()
    except OSError:
        return None
    try:
        p.relative_to(ATLAS_ROOT)
    except ValueError:
        return None
    return p


def _user_wants_file_read(text: str) -> bool:
    t = text.lower()
    if any(x in t for x in READ_TRIGGERS):
        return True
    return bool(re.search(r"(?i)C:\\ATLAS_PUSH\\", text))


def _extract_path_candidates(text: str) -> list[str]:
    found: list[str] = []
    for m in re.finditer(r'C:\\ATLAS_PUSH\\[^\s`"\'<>|*?]+', text, re.I):
        found.append(m.group(0))
    for m in re.finditer(r'[`"]([A-Za-z]:[^`"]+)[`"]', text):
        found.append(m.group(1))
    for m in re.finditer(
        r"(?:^|[\s/\\])([\w][\w./\\-]*\.(?:py|json|md|txt|yaml|yml|toml|sqlite3))\b",
        text,
        re.I,
    ):
        found.append(m.group(1))
    return found


def detect_read_file_target(user_text: str) -> str | None:
    if not _user_wants_file_read(user_text):
        if not re.search(r"(?i)C:\\ATLAS_PUSH\\", user_text):
            return None
    for raw in _extract_path_candidates(user_text):
        if re.match(r"(?i)C:\\ATLAS_PUSH\\", raw):
            p = _safe_under_atlas(raw)
        else:
            p = _safe_under_atlas(str(ATLAS_ROOT / raw.lstrip("/\\")))
        if p and p.is_file():
            return str(p)
    return None


def read_file_via_mcp(abs_path: str) -> str | None:
    data = mcp({"action": "read_file", "params": {"path": abs_path}}, timeout=30)
    if not data or not isinstance(data, dict):
        return None
    return data.get("content")


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


def is_system_status_question(text: str) -> bool:
    msg = text.lower()
    if any(
        w in msg
        for w in (
            "cómo está",
            "como esta",
            "como está",
            "status",
            "funcionando",
            "cómo va",
            "como va",
            "salud del sistema",
        )
    ):
        return True
    if "sistema" in msg and any(
        w in msg for w in ("cómo", "como", "está", "esta", "va ", " va", "salud")
    ):
        return True
    if "atlas" in msg and any(
        w in msg for w in ("estado", "cómo", "como", "va", "ok", "anda")
    ):
        return True
    return False


def ask_llm(user_message: str, atlas_context: str) -> str:
    """Envía el mensaje al LLM local con contexto de ATLAS."""
    global ACTIVE_MODEL

    if not ACTIVE_MODEL:
        return ask_llm_fallback(user_message, atlas_context)

    system_prompt = f"""Eres el asistente operativo de ATLAS, sistema de trading algorítmico en paper mode.
Respondes en español, de forma concisa y directa.
Da siempre los números exactos del contexto. Nunca inventes datos.

REGLAS CRÍTICAS:
- NUNCA describas el estado del sistema sin haber recibido datos reales del MCP. Si la consulta MCP falla, responde exactamente: 'No pude obtener datos en este momento, intenta de nuevo.'
- Si el contexto no tiene el dato que piden, di exactamente: "No tengo ese dato disponible ahora mismo."
- NUNCA inventes valores de PnL, trades, logs, errores o configuraciones.
- Puedes resumir archivos solo si el contexto incluye un bloque "CONTENIDO_ARCHIVO" con texto real leído vía MCP.
- Las acciones que SÍ puedes ejecutar vía MCP cuando el usuario lo pide: pausar/reanudar trading (fail_safe) si el servidor lo soporta, consultar journal, ver estado, ver logs, leer archivos bajo C:\\ATLAS_PUSH.
- Si el MCP no responde, dilo claramente o usa la frase obligatoria indicada arriba para estado del sistema.

CONTEXTO ACTUAL DEL SISTEMA (datos reales, no inventar más allá de esto):
{atlas_context}

Máximo 4-5 líneas. Sin listas largas. Sin rodeos."""

    # Construir historial
    messages_text = ""
    for turn in conversation_history[-MAX_HISTORY:]:
        messages_text += f"Usuario: {turn['user']}\nATLAS: {turn['assistant']}\n"
    messages_text += f"Usuario: {user_message}\nATLAS:"

    payload = {
        "model": ACTIVE_MODEL,
        "prompt": f"<s>[INST] <<SYS>>\n{system_prompt}\n<</SYS>>\n\n{messages_text} [/INST]",
        "stream": False,
        "options": {"temperature": 0.3, "num_predict": 300, "stop": ["Usuario:", "User:"]},
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

    if is_system_status_question(user_message):
        if "MCP no responde" in atlas_context or "sin datos de estado" in atlas_context:
            return "No pude obtener datos en este momento, intenta de nuevo."
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
        result = mcp(
            {"action": "set_state", "params": {"key": "fail_safe_active", "value": True}}
        )
        if result is not None:
            return "Trading pausado. Fail-safe activado — ATLAS no ejecutará nuevas órdenes."
        return "No pude conectar con el MCP para pausar."

    if any(w in msg for w in ["reanudar", "continuar", "resume", "activar", "encender"]):
        result = mcp(
            {"action": "set_state", "params": {"key": "fail_safe_active", "value": False}}
        )
        if result is not None:
            return "Trading reanudado. Fail-safe desactivado."
        return "No pude conectar con el MCP para reanudar."

    if any(w in msg for w in ["error", "fallo", "problema", "issue"]):
        state_wrap = mcp({"action": "get_state"})
        if state_wrap and isinstance(state_wrap, dict):
            state = state_wrap.get("state") or {}
            if isinstance(state, dict):
                n = state.get("operational_error_count", 0)
                last = state.get("last_operational_error") or "Ninguno"
                return f"Errores operacionales: {n}\nÚltimo: {last}"
        return "No pude obtener datos en este momento, intenta de nuevo."

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
            "— Lee el archivo X.py en ATLAS_PUSH"
        )

    # Respuesta genérica con contexto
    return f"Entendido. Estado actual:\n{atlas_context[:400]}"


# ── Acciones ejecutables desde el chat ───────────────────────────────────────
def check_and_execute_action(user_message: str, llm_response: str):
    """Detecta si el usuario pidió una acción y la ejecuta."""
    msg = user_message.lower()

    if any(w in msg for w in ["pausa", "pausar", "detener trading", "stop trading"]):
        mcp(
            {"action": "set_state", "params": {"key": "fail_safe_active", "value": True}}
        )
        log.info("Acción ejecutada: fail_safe=True")

    elif any(w in msg for w in ["reanudar", "reiniciar trading", "resume trading"]):
        mcp(
            {"action": "set_state", "params": {"key": "fail_safe_active", "value": False}}
        )
        log.info("Acción ejecutada: fail_safe=False")


# ── Telegram helpers ──────────────────────────────────────────────────────────
def send(text: str) -> bool:
    try:
        max_len = 4096
        chunk = text[:max_len] if len(text) <= max_len else text[: max_len - 20] + "\n…(truncado)"
        r = requests.post(
            f"{TELEGRAM_URL}/sendMessage",
            json={"chat_id": CHAT_ID, "text": chunk},
            timeout=10,
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
            timeout=5,
        )
    except Exception:
        pass


def get_updates(offset: int = 0) -> list:
    try:
        r = requests.get(
            f"{TELEGRAM_URL}/getUpdates",
            params={"offset": offset, "timeout": 30, "allowed_updates": ["message"]},
            timeout=35,
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


def ensure_polling_only():
    """
    Si hay webhook configurado además de este script (getUpdates), Telegram puede
    entregar el mismo mensaje por dos canales → dos respuestas. Solo polling.
    """
    try:
        r = requests.get(f"{TELEGRAM_URL}/getWebhookInfo", timeout=10)
        if r.ok:
            info = r.json().get("result") or {}
            url = (info.get("url") or "").strip()
            if url:
                log.warning(
                    "Había webhook activo en este bot; se elimina para usar solo long-polling "
                    f"(url={url[:96]}…)."
                )
        r2 = requests.post(
            f"{TELEGRAM_URL}/deleteWebhook",
            json={"drop_pending_updates": False},
            timeout=10,
        )
        if not r2.ok:
            log.warning(f"deleteWebhook no OK: {r2.status_code} {r2.text[:200]}")
    except Exception as e:
        log.warning(f"ensure_polling_only: {e}")


# ── Loop principal ─────────────────────────────────────────────────────────────
def main():
    global ACTIVE_MODEL

    acquire_bot_lock()

    log.info("=" * 60)
    log.info("ATLAS Telegram Bot NLP v3.0 iniciando...")

    ensure_polling_only()

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
                uid = update["update_id"]
                offset = uid + 1
                if not should_process_update(uid):
                    log.info(f"Update {uid} ya procesado; se confirma offset sin reenviar.")
                    continue

                msg = update.get("message", {})
                if not msg or not msg.get("text"):
                    continue

                cid = str(msg.get("chat", {}).get("id", ""))
                if cid != CHAT_ID:
                    continue

                user_text = msg["text"].strip()
                log.info(f"Usuario: '{user_text}'")

                send_typing()

                read_target = detect_read_file_target(user_text)
                if read_target:
                    content = read_file_via_mcp(read_target)
                    if content is None:
                        response = "No pude leer ese archivo vía MCP (error o ruta inválida)."
                    else:
                        snippet = content[:12000]
                        if len(content) > 12000:
                            snippet += "\n…(archivo truncado para el chat)"
                        response = f"Archivo: {read_target}\n\n{snippet}"
                    conversation_history.append({"user": user_text, "assistant": response})
                    if len(conversation_history) > MAX_HISTORY:
                        conversation_history.pop(0)
                    send(response)
                    log.info("Respuesta: lectura de archivo vía MCP")
                    continue

                atlas_ctx, meta = get_atlas_context()

                if is_system_status_question(user_text) and not meta.get("state_ok"):
                    response = "No pude obtener datos en este momento, intenta de nuevo."
                    conversation_history.append({"user": user_text, "assistant": response})
                    if len(conversation_history) > MAX_HISTORY:
                        conversation_history.pop(0)
                    send(response)
                    continue

                response = ask_llm(user_text, atlas_ctx)

                check_and_execute_action(user_text, response)

                conversation_history.append({"user": user_text, "assistant": response})
                if len(conversation_history) > MAX_HISTORY:
                    conversation_history.pop(0)

                send(response)
                log.info(f"ATLAS: '{response[:80]}...'")

        except KeyboardInterrupt:
            log.info("Bot detenido.")
            send("ATLAS Bot offline.")
            break
        except requests.exceptions.HTTPError as e:
            if "409" in str(e):
                log.error("Error 409 Conflict — otra instancia del bot activa. Esperando 15s...")
                time.sleep(15)
            else:
                log.error(f"HTTP error: {e}")
                time.sleep(5)
        except Exception as e:
            log.error(f"Error en loop: {e}")
            time.sleep(5)
    _release_bot_lock()


if __name__ == "__main__":
    try:
        main()
    finally:
        _release_bot_lock()

