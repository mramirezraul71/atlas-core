import os
from dotenv import load_dotenv
from telegram import Update
from telegram.ext import Application, CommandHandler, MessageHandler, ContextTypes, filters

from core.logger import log
from modules.command_router import handle
from modules.atlas_llm import atlas

ENV_PATH = r"C:\ATLAS\config\.env"

def load_env():
    load_dotenv(ENV_PATH)

def env(name: str, default=""):
    return os.getenv(name, default).strip()

async def cmd_start(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text(
        " ATLAS (Telegram) listo.\n"
        "Escribe: help\n"
        "Ejemplos:\n"
        "- status\n"
        "- dir C:\\ATLAS\n"
        "- open_rauli\n"
        "- run dir C:\\ATLAS\\modules"
    )

async def cmd_help(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text(handle("help"))

async def cmd_status(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text(handle("status"))

async def on_text(update: Update, context: ContextTypes.DEFAULT_TYPE):
    msg = (update.message.text or "").strip()
    log(f"TG_IN: {msg}")

    # Ruta 1: comandos locales
    if msg.lower().startswith(("help","status","dir ","open_rauli","run ")):
        resp = handle(msg)
        await update.message.reply_text(resp)
        return

    # Ruta 2: cerebro (LLM) para responder/planificar
    if not atlas.ready():
        await update.message.reply_text(" OPENAI_API_KEY no está cargada. Revisa C:\\ATLAS\\config\\.env")
        return

    system = (
        "Eres ATLAS, asistente del usuario. Responde en español. "
        "Sé directo. Si el usuario pide ejecutar algo, sugiere el comando 'run ...' "
        "o un comando permitido."
    )
    try:
        resp = atlas.think(msg, system=system)
        if not resp:
            resp = "(sin respuesta)"
        if len(resp) > 3500:
            resp = "(recortado)\n" + resp[-3500:]
        await update.message.reply_text(resp)
    except Exception as e:
        await update.message.reply_text(f"ERROR LLM: {e}")

def main():
    load_env()
    token = env("TELEGRAM_BOT_TOKEN")
    if not token:
        print("FALTA: TELEGRAM_BOT_TOKEN en C:\\ATLAS\\config\\.env")
        return

    app = Application.builder().token(token).build()
    app.add_handler(CommandHandler("start", cmd_start))
    app.add_handler(CommandHandler("help", cmd_help))
    app.add_handler(CommandHandler("status", cmd_status))
    app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, on_text))

    print("ATLAS Telegram ON. (Ctrl+C para salir)")
    log("ATLAS_TELEGRAM: ON")
    app.run_polling()

if __name__ == "__main__":
    main()
