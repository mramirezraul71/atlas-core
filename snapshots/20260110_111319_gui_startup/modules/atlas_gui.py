# -*- coding: utf-8 -*-
import os
import threading
import subprocess
import tkinter as tk
from tkinter import scrolledtext
from datetime import datetime

import pystray
from pystray import MenuItem as item
from PIL import Image, ImageDraw

from core.logger import log
from modules.snapshot_engine import snapshot

HELP = """Comandos ATLAS:
  help                 -> ayuda
  status               -> estado rapido
  doctor               -> flutter doctor -v (se guarda en log)
  snapshot [label]     -> crear snapshot
  run rauli            -> abre RAULI (flutter run -d windows)
  exit                 -> cerrar ATLAS
"""

def now():
    return datetime.now().strftime("%H:%M:%S")

def status():
    return "ATLAS OK | logs=C:\\ATLAS\\logs\\atlas.log | snapshots=C:\\ATLAS\\snapshots"

def _run_shell(cmd, cwd=None):
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, shell=True)
    out = (p.stdout or "").strip()
    err = (p.stderr or "").strip()
    return out, err

def doctor():
    log("ATLAS GUI: doctor solicitado")
    out, err = _run_shell("flutter doctor -v")
    if out:
        log(out)
    if err:
        log("ERROR doctor:")
        log(err)
    return "Doctor ejecutado. Revisa el log: C:\\ATLAS\\logs\\atlas.log"

def run_rauli():
    app = r"C:\ATLAS\rauli_core_app"
    if not os.path.exists(app):
        return "No existe C:\\ATLAS\\rauli_core_app. (Tu proyecto Flutter real debe estar ahí.)"
    log("ATLAS GUI: run rauli solicitado")
    subprocess.Popen("flutter run -d windows", cwd=app, shell=True)
    return "RAULI lanzado (flutter run -d windows)."

def handle(cmd: str):
    cmd = (cmd or "").strip()
    if not cmd:
        return ""
    low = cmd.lower().strip()

    if low in ("help", "?"):
        return HELP.strip()

    if low == "status":
        return status()

    if low.startswith("snapshot"):
        parts = cmd.split(maxsplit=1)
        label = parts[1] if len(parts) > 1 else "manual"
        path = snapshot(label)
        return f"Snapshot creado: {path}"

    if low == "doctor":
        return doctor()

    if low == "run rauli":
        return run_rauli()

    if low in ("exit", "quit", "salir"):
        return "__EXIT__"

    return "Comando no reconocido. Escribe: help"

class AtlasGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ATLAS Assistant")
        self.root.geometry("820x520")
        self.root.protocol("WM_DELETE_WINDOW", self.hide_window)

        top = tk.Frame(self.root)
        top.pack(fill="x", padx=10, pady=8)

        self.status_lbl = tk.Label(top, text="ATLAS listo", anchor="w")
        self.status_lbl.pack(side="left", fill="x", expand=True)

        self.btn_snapshot = tk.Button(top, text="Snapshot", command=self.quick_snapshot)
        self.btn_snapshot.pack(side="right", padx=(8,0))

        self.btn_doctor = tk.Button(top, text="Doctor", command=self.quick_doctor)
        self.btn_doctor.pack(side="right", padx=(8,0))

        self.chat = scrolledtext.ScrolledText(self.root, wrap="word", height=18)
        self.chat.pack(fill="both", expand=True, padx=10, pady=(0,10))
        self.chat.configure(state="disabled")

        bottom = tk.Frame(self.root)
        bottom.pack(fill="x", padx=10, pady=(0,10))

        self.entry = tk.Entry(bottom)
        self.entry.pack(side="left", fill="x", expand=True)
        self.entry.bind("<Return>", lambda e: self.send())

        self.send_btn = tk.Button(bottom, text="Enviar", command=self.send)
        self.send_btn.pack(side="right", padx=(8,0))

        self._append(f"[{now()}] ATLAS: Ventana lista. Escribe 'help' para comandos.")
        log("ATLAS GUI iniciado")
        snapshot("gui_startup")

        self.tray = None
        self._start_tray()

    def _append(self, text):
        self.chat.configure(state="normal")
        self.chat.insert("end", text + "\n")
        self.chat.see("end")
        self.chat.configure(state="disabled")

    def send(self):
        cmd = self.entry.get().strip()
        if not cmd:
            return
        self.entry.delete(0, "end")
        self._append(f"[{now()}] Tú: {cmd}")

        resp = handle(cmd)
        if resp == "__EXIT__":
            self._append(f"[{now()}] ATLAS: Cerrando")
            log("ATLAS GUI: salida solicitada")
            self.quit_all()
            return

        if resp:
            self._append(f"[{now()}] ATLAS: {resp}")
            log(f"ATLAS GUI cmd='{cmd}' resp='{resp}'")

    def quick_snapshot(self):
        p = snapshot("quick_gui")
        self._append(f"[{now()}] ATLAS: Snapshot creado: {p}")

    def quick_doctor(self):
        r = doctor()
        self._append(f"[{now()}] ATLAS: {r}")

    # -------- Tray --------
    def _icon_image(self):
        img = Image.new("RGBA", (64, 64), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        d.rounded_rectangle((6, 6, 58, 58), radius=14, fill=(0, 128, 128, 255))
        d.text((18, 18), "A", fill=(255,255,255,255))
        return img

    def _start_tray(self):
        def run_tray():
            menu = (
                item("Mostrar ATLAS", self.show_window),
                item("Ocultar", self.hide_window),
                item("Snapshot", lambda: self.quick_snapshot()),
                item("Salir", lambda: self.quit_all()),
            )
            self.tray = pystray.Icon("ATLAS", self._icon_image(), "ATLAS Assistant", menu)
            self.tray.run()

        t = threading.Thread(target=run_tray, daemon=True)
        t.start()

    def show_window(self, *args):
        self.root.after(0, lambda: (self.root.deiconify(), self.root.lift()))

    def hide_window(self, *args):
        self.root.after(0, lambda: self.root.withdraw())

    def quit_all(self, *args):
        try:
            log("ATLAS GUI finalizando")
        except:
            pass
        try:
            if self.tray:
                self.tray.stop()
        except:
            pass
        try:
            self.root.destroy()
        except:
            pass
        os._exit(0)

    def run(self):
        self.root.mainloop()

def main():
    gui = AtlasGUI()
    gui.run()

if __name__ == "__main__":
    main()
