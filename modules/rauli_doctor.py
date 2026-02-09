from pathlib import Path
from datetime import datetime
import os

def _env(name: str, default: str) -> str:
    v = os.getenv(name)
    return v.strip() if v and v.strip() else default

ATLAS_ROOT = Path(_env("ATLAS_ROOT", r"C:\ATLAS"))
VAULT_DIR  = Path(_env("ATLAS_VAULT_DIR", str(ATLAS_ROOT / "ATLAS_VAULT")))
NOTES_DIR  = Path(_env("ATLAS_NOTES_DIR", str(VAULT_DIR / "NOTES")))
LOGS_DIR   = Path(_env("ATLAS_LOGS_DIR", str(ATLAS_ROOT / "logs")))
SNAPS_DIR  = Path(_env("ATLAS_SNAPS_DIR", str(ATLAS_ROOT / "snapshots")))

def doctor_report() -> str:
    parts = [("VAULT", VAULT_DIR), ("NOTES", NOTES_DIR), ("LOGS", LOGS_DIR), ("SNAPS", SNAPS_DIR)]
    lines = ["ATLAS DOCTOR:"]
    ok = True

    for name, p in parts:
        try:
            p.mkdir(parents=True, exist_ok=True)
            test = p / ".atlas_write_test"
            test.write_text("ok", encoding="utf-8")
            test.unlink(missing_ok=True)
            lines.append(f"- {name}: OK")
        except Exception as e:
            ok = False
            lines.append(f"- {name}: FAIL ({e})")

    if ok:
        lines.append(f" OK ({datetime.now().strftime('%Y-%m-%d %H:%M:%S')})")
    else:
        lines.append(f" FAIL ({datetime.now().strftime('%Y-%m-%d %H:%M:%S')})")

    return "\n".join(lines)

def run_doctor() -> str:
    # Para GUI: devuelve texto listo para mostrar
    return doctor_report()

if __name__ == "__main__":
    print(doctor_report())
