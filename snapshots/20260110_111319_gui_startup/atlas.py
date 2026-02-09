# -*- coding: utf-8 -*-

"""
ATLAS  Asistente Digital Local
Cuerpo + Cerebro + Memoria + Automatizacion
"""

import os
import time
import psutil
import threading
import pyttsx3
import requests
from datetime import datetime

# =========================
# VOZ
# =========================
engine = pyttsx3.init()
engine.setProperty("rate", 175)

def hablar(texto):
    print(f"ATLAS: {texto}")
    engine.say(texto)
    engine.runAndWait()

# =========================
# MEMORIA SIMPLE
# =========================
MEMORY_FILE = r"C:\ATLAS\memory\atlas_memory.txt"
os.makedirs(r"C:\ATLAS\memory", exist_ok=True)

def recordar(texto):
    with open(MEMORY_FILE, "a", encoding="utf-8") as f:
        f.write(f"[{datetime.now()}] {texto}\n")

# =========================
# ESTADO DEL SISTEMA
# =========================
def estado():
    cpu = psutil.cpu_percent(interval=1)
    ram = psutil.virtual_memory().percent
    return f"CPU {cpu}% | RAM {ram}%"

# =========================
# COMANDOS
# =========================
def procesar(comando: str):
    comando = comando.lower().strip()

    if comando in ("hola", "atlas"):
        hablar("Aqui estoy. Dime que necesitas.")
        return

    if "estado" in comando:
        s = estado()
        hablar(s)
        recordar(f"Estado solicitado: {s}")
        return

    if comando.startswith("recordar"):
        txt = comando.replace("recordar", "").strip()
        recordar(txt)
        hablar("Memoria guardada.")
        return

    if comando == "salir":
        hablar("ATLAS apagandose.")
        os._exit(0)

    hablar("Comando no reconocido.")

# =========================
# LOOP PRINCIPAL (CHAT)
# =========================
def main():
    hablar("ATLAS iniciado. Sistema operativo.")
    while True:
        try:
            cmd = input("atlas> ")
            procesar(cmd)
        except KeyboardInterrupt:
            hablar("ATLAS detenido manualmente.")
            break

if __name__ == "__main__":
    main()
