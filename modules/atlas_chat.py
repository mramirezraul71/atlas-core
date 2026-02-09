# -*- coding: utf-8 -*-
import os
import subprocess
from core.logger import log
from modules.snapshot_engine import snapshot

HELP = """
Comandos:
  help                 -> ver ayuda
  status               -> estado rapido
  doctor               -> ejecuta RAULI Doctor
  snapshot [label]     -> crea snapshot
  run rauli            -> abre RAULI (flutter run -d windows) desde el proyecto
  exit                 -> salir
"""

def _run_shell(cmd, cwd=None):
    try:
        p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, shell=True)
        out = (p.stdout or "").strip()
        err = (p.stderr or "").strip()
        return out, err
    except Exception as e:
        return "", str(e)

def status():
    return "ATLAS OK | logs=C:\\ATLAS\\logs\\atlas.log | snapshots=C:\\ATLAS\\snapshots"

def doctor():
    log("ATLAS CHAT: doctor solicitado")
    out, err = _run_shell("flutter doctor -v")
    if out:
        log(out)
    if err:
        log("ERROR doctor:")
        log(err)
    return "Doctor ejecutado. Revisa el log: C:\\ATLAS\\logs\\atlas.log"

def run_rauli():
    # Proyecto Flutter real (el que creaste)
    app = r"C:\ATLAS\rauli_core_app"
    if not os.path.exists(app):
        return "No existe C:\\ATLAS\\rauli_core_app. Primero crea/corre el proyecto Flutter."
    log("ATLAS CHAT: run rauli solicitado")
    # Esto lanza Flutter; no captura salida en vivo (es intencional)
    subprocess.Popen("flutter run -d windows", cwd=app, shell=True)
    return "RAULI lanzado (flutter run -d windows)."

def handle(cmd: str):
    cmd = cmd.strip()
    if not cmd:
        return ""
    if cmd in ("help", "?"):
        return HELP.strip()
    if cmd == "status":
        return status()
    if cmd.startswith("snapshot"):
        parts = cmd.split(maxsplit=1)
        label = parts[1] if len(parts) > 1 else "manual"
        snap = snapshot(label)
        return f"Snapshot creado: {snap}"
    if cmd == "doctor":
        return doctor()
    if cmd == "run rauli":
        return run_rauli()
    if cmd in ("exit", "quit"):
        return "__EXIT__"
    return "Comando no reconocido. Escribe: help"

def main():
    print("ATLAS CHAT listo. Escribe: help")
    while True:
        try:
            cmd = input("atlas> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        resp = handle(cmd)
        if resp == "__EXIT__":
            break
        if resp:
            print(resp)

if __name__ == "__main__":
    main()
