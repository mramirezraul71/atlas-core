import os
import re
import json
from datetime import datetime
from pathlib import Path

from dotenv import load_dotenv

# python-telegram-bot v20+
from telegram import Update
from telegram.constants import ParseMode
from telegram.ext import Application, CommandHandler, MessageHandler, ContextTypes, filters

# Importa tu LLM (ya lo probaste con test_llm.py)
from modules.atlas_llm import atlas

BASE = Path(r"C:\ATLAS")
VAULT = BASE / "ATLAS_VAULT"
NOTES = VAULT / "NOTES"
LOGS = BASE / "logs"
SNAPS = BASE / "snapshots"

def now_ts() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def safe_filename(name: str) -> str:
    name = name.strip()
    name = re.sub(r'[<>:"/\\|?*]', "_", name)
    name = re.sub(r"\s+", " ", name)
    return name

def write_note(title: str, body: str = "") -> Path:
    NOTES.mkdir(parents=True, exist_ok=True)
    fn = safe_filename(title)
    p = NOTES / f"{fn}.md"
    if not p.exists():
        p.write_text(f"# {title}\n\nCreado: {now_ts()}\n\n{body}\n", encoding="utf-8")
    else:
        # si existe, agrega abajo
        with p.open("a", encoding="utf-8") as f:
            f.write(f"\n---\nActualizado: {now_ts()}\n\n{body}\n")
    return p

def status_text() -> str:
    return f"ATLAS: OK | logs={LOGS}\\atlas.log | snapshots={SNAPS}"

def doctor_text() -> str:
    # doctor simple sin depender de otros módulos
    checks = []
    checks.append(("VAULT", VAULT.exists()))
    checks.append(("NOTES", NOTES.exists()))
    checks.append(("LOGS", LOGS.exists()))
    checks.append(("SNAPS", SNAPS.exists()))
    ok = all(v for _, v in checks)
    lines = ["ATLAS DOCTOR: " + ("OK " if ok else "OBSERVACIONES ")]
    for k,v in checks:
        lines.append(f"- {k}: {'OK' if v else 'NO'}")
    return "\n".join(lines)

def snapshot(label: str = "manual") -> Path:
    SNAPS.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = SNAPS / f"{stamp}_{safe_filename(label)}"
    folder.mkdir(parents=True, exist_ok=True)
    # mínimo: guardamos un marker
    (folder / "SNAPSHOT.txt").write_text(f"Snapshot: {stamp}\nLabel: {label}\n", encoding="utf-8")
    return folder

SYSTEM_NLP = """Eres ATLAS, un router de acciones para un asistente local.
Convierte el texto del usuario en UNA acción JSON estricta, sin explicación.
Acciones permitidas:
- note: crear/actualizar una nota. Requiere "title". Opcional "body".
- status: estado rápido.
- doctor: verificación de salud.
- snapshot: snapshot. Opcional "label".
Si no corresponde a nada, usa {"action":"note","title":"Inbox","body":"<texto original>"} para no perderlo.
Responde SOLO JSON."""

def nlp_to_action(user_text: str) -> dict:
    # Llama al modelo y exige JSON
    raw = atlas.think(user_text, system=SYSTEM_NLP)
    try:
        obj = json.loads(raw)
        if "action" not in obj:
            raise ValueError("no action")
        return obj
    except Exception:
        return {"action": "note", "title": "Inbox", "body": user_text}

async def cmd_start(update: Update, context: ContextTypes.DEFAULT_TYPE):
    msg = (
        "ATLAS Telegram listo.\n"
        "Comandos:\n"
        "/status\n"
        "/doctor\n"
        "/snapshot etiqueta\n\n"
        "También puedes escribir en lenguaje natural:\n"
        "Ej: 'Atlas, crea una nota llamada Visión ATLAS-RAULI'"
    )
    await update.message.reply_text(msg)

async def cmd_status(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text(status_text())

async def cmd_doctor(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text(doctor_text())

async def cmd_snapshot(update: Update, context: ContextTypes.DEFAULT_TYPE):
    label = "manual"
    if context.args:
        label = " ".join(context.args).strip()
    p = snapshot(label)
    await update.message.reply_text(f"Snapshot creado: {p}")

async def on_text(update: Update, context: ContextTypes.DEFAULT_TYPE):
    text = (update.message.text or "").strip()
    if not text:
        return

    # Si el usuario escribe /algo, deja que Telegram lo trate como comando
    if text.startswith("/"):
        return

    # NLP -> acción
    act = nlp_to_action(text)
    action = (act.get("action") or "").lower()

    if action == "status":
        await update.message.reply_text(status_text())
        return

    if action == "doctor":
        await update.message.reply_text(doctor_text())
        return

    if action == "snapshot":
        label = act.get("label") or "nlp"
        p = snapshot(str(label))
        await update.message.reply_text(f"Snapshot creado: {p}")
        return

    if action == "note":
        title = act.get("title") or "Inbox"
        body = act.get("body") or ""
        p = write_note(str(title), str(body))
        await update.message.reply_text(f"Nota OK \n{p}")
        return

    # fallback seguro
    p = write_note("Inbox", text)
    await update.message.reply_text(f"Guardado en Inbox \n{p}")

def main():
    load_dotenv(str(BASE / "config" / ".env"))

    token = os.getenv("TELEGRAM_BOT_TOKEN", "").strip()
    if not token:
        raise RuntimeError("Falta TELEGRAM_BOT_TOKEN en C:\\ATLAS\\config\\.env")

    app = Application.builder().token(token).build()

    app.add_handler(CommandHandler("start", cmd_start))
    app.add_handler(CommandHandler("status", cmd_status))
    app.add_handler(CommandHandler("doctor", cmd_doctor))
    app.add_handler(CommandHandler("snapshot", cmd_snapshot))

    app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, on_text))

    print("ATLAS_TELEGRAM: corriendo...")
    app.run_polling()

if __name__ == "__main__":
    main()
