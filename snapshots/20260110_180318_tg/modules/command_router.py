import os
import subprocess
from core.logger import log

ALLOWED = {
    "status": "Estado rápido",
    "open_rauli": "Abre RAULI (flutter run -d windows) desde C:\\ATLAS\\rauli_core_app si existe",
    "dir": "Lista una carpeta (dir <ruta>)",
    "run": "Ejecuta comando permitido (run <comando>)",
}

SAFE_RUN_PREFIXES = [
    "dir ",
    "cd ",
    "flutter ",
    "python ",
    "pip ",
    "git ",
]

def help_text():
    lines = ["Comandos:"]
    for k,v in ALLOWED.items():
        lines.append(f"- {k}: {v}")
    lines.append("")
    lines.append("Uso:")
    lines.append("dir C:\\ATLAS")
    lines.append("open_rauli")
    lines.append("status")
    return "\n".join(lines)

def _safe(cmd: str) -> bool:
    c = cmd.strip().lower()
    return any(c.startswith(p) for p in SAFE_RUN_PREFIXES)

def run_shell(cmd: str) -> str:
    if not _safe(cmd):
        return " Comando bloqueado por seguridad. Solo se permiten prefijos: " + ", ".join(SAFE_RUN_PREFIXES)

    try:
        log(f"RUN: {cmd}")
        # ejecuta desde C:\ATLAS por defecto
        p = subprocess.run(cmd, shell=True, cwd=r"C:\ATLAS", capture_output=True, text=True)
        out = (p.stdout or "") + (p.stderr or "")
        out = out.strip()
        if not out:
            out = "(sin salida)"
        # recorta para Telegram
        if len(out) > 3500:
            out = out[-3500:]
            out = "(recortado)\n" + out
        return out
    except Exception as e:
        return f"ERROR: {e}"

def handle(text: str) -> str:
    t = (text or "").strip()
    if not t:
        return "Escribe 'help' para ver comandos."
    low = t.lower()

    if low == "help":
        return help_text()

    if low == "status":
        return " ATLAS OK | logs=C:\\ATLAS\\logs\\atlas.log"

    if low.startswith("dir "):
        return run_shell("dir " + t[4:].strip())

    if low == "open_rauli":
        # intenta correr RAULI si existe
        app = r"C:\ATLAS\rauli_core_app"
        if not os.path.isdir(app):
            return "No veo C:\\ATLAS\\rauli_core_app. Crea/ubica tu proyecto ahí y probamos otra vez."
        return run_shell("flutter run -d windows")

    if low.startswith("run "):
        return run_shell(t[4:].strip())

    return "Comando no reconocido. Escribe 'help'."
