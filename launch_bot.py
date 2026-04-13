"""
Arranque del bot NLP: carga variables desde C:\\ATLAS_PUSH\\.env (ignorado por git).
No incluir tokens en este archivo.
"""

import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ENV_PATH = ROOT / ".env"


def _load_env_file(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.is_file():
        return out
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, _, v = line.partition("=")
            out[k.strip()] = v.strip()
    return out


def main() -> None:
    os.system('taskkill /F /FI "WINDOWTITLE eq ATLAS*" /IM python.exe >nul 2>&1')

    file_vars = _load_env_file(ENV_PATH)
    env = os.environ.copy()
    for k, v in file_vars.items():
        env.setdefault(k, v)

    if not (env.get("ATLAS_BOT_TOKEN") or env.get("BOT_TOKEN")):
        print("FALLO: falta ATLAS_BOT_TOKEN o BOT_TOKEN en .env", file=sys.stderr)
        sys.exit(1)
    if not (env.get("ATLAS_CHAT_ID") or env.get("CHAT_ID")):
        print("FALLO: falta ATLAS_CHAT_ID o CHAT_ID en .env", file=sys.stderr)
        sys.exit(1)

    env.setdefault("ATLAS_MCP_URL", "http://localhost:8799/execute")
    env.setdefault(
        "ATLAS_JOURNAL_DB",
        str(ROOT / "atlas_code_quant" / "data" / "journal" / "trading_journal.sqlite3"),
    )
    env.setdefault("ATLAS_OLLAMA_URL", "http://localhost:11434/api/generate")

    log_path = ROOT / "telegram_bot_nlp.log"
    log_file = open(log_path, "a", encoding="utf-8")

    proc = subprocess.Popen(
        [sys.executable, str(ROOT / "atlas_telegram_bot_nlp.py")],
        cwd=str(ROOT),
        env=env,
        stdout=log_file,
        stderr=log_file,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
    )

    time.sleep(4)
    if proc.poll() is None:
        print(f"OK PID={proc.pid}")
    else:
        log_file.close()
        tail = log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-12:]
        print("FALLO. Ultimas lineas del log:")
        for line in tail:
            print(line.rstrip())


if __name__ == "__main__":
    main()
