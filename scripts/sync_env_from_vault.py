#!/usr/bin/env python3
"""
Sincroniza .env en la raíz del repo desde la Bóveda (credenciales.txt).
Lee C:\\dev\\credenciales.txt (o ATLAS_VAULT_PATH) y escribe solo las variables
necesarias para ATLAS en .env. No imprime claves.
Uso: python scripts/sync_env_from_vault.py
"""
import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
ENV_KEYS = [
    "ATLAS_VAULT_PATH",
    "OLLAMA_BASE_URL",
    "OLLAMA_MODEL",
    "OLLAMA_MODEL_FAST",
    "OLLAMA_MODEL_CHAT",
    "OLLAMA_MODEL_CODE",
    "OLLAMA_MODEL_REASON",
    "OLLAMA_MODEL_TOOLS",
    "OPENAI_API_KEY",
    "ANTHROPIC_API_KEY",
    "GEMINI_API_KEY",
    "GROQ_API_KEY",
    "DEEPSEEK_API_KEY",
    "XAI_API_KEY",
    "MISTRAL_API_KEY",
    "PERPLEXITY_API_KEY",
    "AWS_ACCESS_KEY_ID",
    "AWS_SECRET_ACCESS_KEY",
    "AWS_DEFAULT_REGION",
    "AWS_REGION",
    "CREDENTIALS_FILE",
    "ATLAS_AI_MODE",
    "ATLAS_MODEL_TUTOR",
    "ATLAS_MODEL_FAST",
    "ANTHROPIC_API_KEY_CEREBRO",
]
# Si la Bóveda tiene ANTHROPIC_API_KEY_CEREBRO y no ANTHROPIC_API_KEY, mapear.
ANTHROPIC_FALLBACK_KEY = "ANTHROPIC_API_KEY_CEREBRO"
BOILERPLATE = """# ATLAS .env — generado por scripts/sync_env_from_vault.py (desde Bóveda)
# No commitear.

"""


def main():
    vault = os.getenv("ATLAS_VAULT_PATH", r"C:\dev\credenciales.txt").strip()
    vault_path = Path(vault)
    if not vault_path.is_file():
        vault_path = REPO_ROOT / "config" / "atlas.env"
    env_out = REPO_ROOT / ".env"
    env_lines = []
    seen = set()

    def add(key: str, value: str):
        if key in seen or not value:
            return
        seen.add(key)
        v = value.replace("\\", "\\\\").replace('"', '\\"')
        env_lines.append(f'{key}="{v}"')

    if vault_path.is_file():
        with open(vault_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, _, v = line.partition("=")
                k, v = k.strip(), v.strip().strip('"').strip("'")
                if k in ENV_KEYS:
                    add(k, v)
                if k == ANTHROPIC_FALLBACK_KEY and v:
                    add("ANTHROPIC_API_KEY", v)

    for k in ENV_KEYS:
        v = (os.getenv(k) or "").strip()
        if v:
            add(k, v)

    if not env_lines:
        add(
            "ATLAS_VAULT_PATH",
            vault if Path(vault).is_file() else r"C:\dev\credenciales.txt",
        )
        add("OLLAMA_BASE_URL", "http://127.0.0.1:11434")
        add("AI_ALLOW_EXTERNAL_APIS", "true")

    try:
        env_out.write_text(BOILERPLATE + "\n".join(env_lines) + "\n", encoding="utf-8")
        print(f"OK: {len(env_lines)} variables escritas en {env_out}")
    except Exception as e:
        print(f"Error escribiendo .env: {e}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
