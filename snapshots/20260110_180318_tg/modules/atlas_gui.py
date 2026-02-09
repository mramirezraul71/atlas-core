import tkinter as tk
from tkinter import scrolledtext
from datetime import datetime
from pathlib import Path
import threading
import subprocess
import os

from dotenv import load_dotenv
from modules.rauli_doctor import run_doctor
from modules.snapshot_engine import snapshot

LOG_FILE = Path(r"C:\ATLAS\logs\atlas.log")

def log(msg: str):
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(f"[{ts}] {msg}\n")

class AtlasGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ATLAS Assistant")
        self.root.geometry("900x550")

        self.top = tk.Frame(self.root)
        self.top.pack(fill="x", padx=8, pady=6)

        self.status_lbl = tk.Label(self.top, text="ATLAS listo", anchor="w")
        self.status_lbl.pack(side="left")

        tk.Button(self.top, text="Doctor", command=self._doctor).pack(side="right", padx=4)
        tk.Button(self.top, text="Snapshot", command=self._snapshot).pack(side="right", padx=4)

        self.text = scrolledtext.ScrolledText(self.root, wrap="word")
        self.text.pack(fill="both", expand=True, padx=8, pady=6)

        self.entry = tk.Entry(self.root)
        self.entry.pack(fill="x", padx=8, pady=(0,6))
        self.entry.bind("<Return>", lambda e: self._send())

        self.btn = tk.Button(self.root, text="Enviar", command=self._send)
        self.btn.pack(anchor="e", padx=8, pady=(0,8))

        self._write("ATLAS: Ventana lista. Escribe 'help' para comandos.\n")
        log("ATLAS_GUI: iniciada")

    def _write(self, s: str):
        self.text.insert("end", s)
        self.text.see("end")

    def _send(self):
        cmd = self.entry.get().strip()
        if not cmd:
            return
        self.entry.delete(0, "end")
        self._write(f"Tú: {cmd}\n")
        self._handle(cmd)

    def _handle(self, cmd: str):
        c = cmd.lower()

        if c in ("help", "?"):
            self._write(
                "ATLAS: Comandos:\n"
                "  help                 -> ayuda\n"
                "  status               -> estado rapido\n"
                "  doctor               -> flutter doctor -v\n"
                "  snapshot [label]     -> crea snapshot\n"
                "  run rauli            -> ejecuta flutter app (windows)\n"
                "  exit                 -> cerrar\n\n"
            )
            return

        if c == "status":
            self._write("ATLAS: OK | logs=C:\\ATLAS\\logs\\atlas.log | snapshots=C:\\ATLAS\\snapshots\n")
            return

        if c.startswith("snapshot"):
            parts = cmd.split(maxsplit=1)
            label = parts[1].strip() if len(parts) > 1 else "manual"
            self._snapshot(label)
            return

        if c == "doctor":
            self._doctor()
            return

        if c == "run rauli":
            self._run_rauli()
            return

        if c == "exit":
            self.root.destroy()
            return

        self._write("ATLAS: Comando no reconocido. Escribe: help\n")

    def _doctor(self):
        self._write("ATLAS: Ejecutando doctor...\n")
        def job():
            try:
                out = run_doctor()
                self._write(out + "\n\n")
            except Exception as e:
                self._write(f"ERROR doctor: {e}\n")
                log(f"ERROR doctor: {e}")
        threading.Thread(target=job, daemon=True).start()

    def _snapshot(self, label="manual"):
        try:
            p = snapshot(label)
            self._write(f"ATLAS: Snapshot creado: {p}\n\n")
        except Exception as e:
            self._write(f"ERROR snapshot: {e}\n")
            log(f"ERROR snapshot: {e}")

    def _run_rauli(self):
        self._write("ATLAS: Lanzando RAULI (flutter run -d windows)...\n")
        def job():
            try:
                # Ajusta esta ruta si tu proyecto está en otro lado:
                proj = Path(r"C:\ATLAS\rauli_core_app")
                if not proj.exists():
                    self._write("ATLAS: No encuentro C:\\ATLAS\\rauli_core_app\n")
                    return
                subprocess.Popen(["cmd", "/c", "flutter run -d windows"], cwd=str(proj))
                self._write("ATLAS: Comando lanzado. Revisa ventana de Flutter.\n\n")
            except Exception as e:
                self._write(f"ERROR run rauli: {e}\n")
                log(f"ERROR run rauli: {e}")
        threading.Thread(target=job, daemon=True).start()

def main():
    # Carga config
    load_dotenv(r"C:\ATLAS\config\.env")
    AtlasGUI().root.mainloop()

if __name__ == "__main__":
    main()
