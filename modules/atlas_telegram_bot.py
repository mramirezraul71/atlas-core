import os
from dotenv import load_dotenv
from telegram import Update
from telegram.ext import Application, CommandHandler, MessageHandler, ContextTypes, filters

from modules.rauli_doctor import run_doctor
from modules.snapshot_engine import snapshot

load_dotenv(r"C:\ATLAS\config\.env")

TOKEN = os.getenv("TELEGRAM_BOT_TOKEN", "").strip()

async def start(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text(
        "ATLAS Telegram listo.\n"
        "Comandos:\n"
        "/status\n"
        "/doctor\n"
        "/snapshot etiqueta\n"
    )

async def status(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text("ATLAS: OK | logs=C:\\ATLAS\\logs\\atlas.log | snapshots=C:\\ATLAS\\snapshots")

async def doctor(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text("Corriendo doctor... (puede tardar)")
    out = run_doctor()
    # Telegram limita mensaje: recortamos
    if len(out) > 3500:
        out = out[:3500] + "\n...(recortado)"
    await update.message.reply_text(out)

async def snap(update: Update, context: ContextTypes.DEFAULT_TYPE):
    label = "tg"
    if context.args:
        label = "_".join(context.args)[:40]
    p = snapshot(label)
    await update.message.reply_text(f"Snapshot creado: {p}")

def main():
    if not TOKEN or ":" not in TOKEN:
        raise SystemExit("TELEGRAM_BOT_TOKEN inválido. Revisa C:\\ATLAS\\config\\.env")

    app = Application.builder().token(TOKEN).build()
    app.add_handler(CommandHandler("start", start))
    app.add_handler(CommandHandler("status", status))
    app.add_handler(CommandHandler("doctor", doctor))
    app.add_handler(CommandHandler("snapshot", snap))

    app.run_polling()

if __name__ == "__main__":
    main()
