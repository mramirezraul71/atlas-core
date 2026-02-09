from datetime import datetime
import subprocess
import sys
from pathlib import Path

def _run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True, shell=True)
    out = (p.stdout or "") + (p.stderr or "")
    return out.strip()

def run_doctor() -> str:
    lines = []
    lines.append(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] RAULI DOCTOR")
    lines.append("---- PYTHON ----")
    lines.append(_run("python --version"))
    lines.append(_run("where python"))

    lines.append("---- FLUTTER ----")
    lines.append(_run("flutter --version"))
    lines.append(_run("where flutter"))

    lines.append("---- DART ----")
    lines.append(_run("dart --version"))
    lines.append(_run("where dart"))

    return "\n".join([x for x in lines if x])

if __name__ == "__main__":
    print(run_doctor())
