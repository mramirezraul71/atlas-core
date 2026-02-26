#!/usr/bin/env python3
"""
Verificación del flujo ATLAS: valida que el router de modelos funciona con
Grok (xAI), DeepSeek y AWS Bedrock. Ejecuta tres tareas mínimas de forma secuencial.
Uso: python scripts/verify_atlas_flow.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = REPO_ROOT / "scripts"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

# Cargar Bóveda / .env antes de importar el router
VAULT = os.getenv("ATLAS_VAULT_PATH", r"C:\dev\credenciales.txt")
for p in [Path(VAULT), REPO_ROOT / ".env", REPO_ROOT / "config" / "atlas.env"]:
    if p.is_file():
        try:
            with open(p, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#") or "=" not in line:
                        continue
                    k, _, v = line.partition("=")
                    k, v = k.strip(), v.strip().strip('"').strip("'")
                    if k and os.getenv(k) in (None, ""):
                        os.environ[k] = v
        except Exception:
            pass
        break

from atlas_model_router import (DEEPSEEK_ENDPOINT, _call_bedrock,
                                _call_openai_compat)


def test_grok() -> tuple[bool, str]:
    """Pequeña tarea con Grok (xAI)."""
    key = (os.getenv("XAI_API_KEY") or "").strip()
    if not key:
        return False, "XAI_API_KEY no configurada"
    ok, text, _ = _call_openai_compat(
        "https://api.x.ai/v1/chat/completions",
        "grok-3-mini",
        "Responde en una sola palabra: OK",
        None,
        key,
        30,
        "xAI",
    )
    return ok, (text[:80] if text else "sin respuesta")


def test_deepseek() -> tuple[bool, str]:
    """Pequeña tarea con DeepSeek."""
    key = (os.getenv("DEEPSEEK_API_KEY") or "").strip()
    if not key:
        return False, "DEEPSEEK_API_KEY no configurada"
    ok, text, _ = _call_openai_compat(
        DEEPSEEK_ENDPOINT,
        "deepseek-chat",
        "Responde en una sola palabra: OK",
        None,
        key,
        30,
        "DeepSeek",
    )
    return ok, (text[:80] if text else "sin respuesta")


def test_bedrock() -> tuple[bool, str]:
    """Pequeña tarea con AWS Bedrock (Claude)."""
    if not (os.getenv("AWS_ACCESS_KEY_ID") and os.getenv("AWS_SECRET_ACCESS_KEY")):
        return False, "Credenciales AWS no configuradas"
    model_id = os.getenv(
        "ATLAS_MODEL_FAST", "us.anthropic.claude-haiku-4-5-20251001-v1:0"
    )
    ok, text, _ = _call_bedrock(
        "Responde en una sola palabra: OK",
        None,
        model_id,
        45,
    )
    return ok, (text[:80] if text else "sin respuesta")


def main() -> int:
    print("=== Verificación del flujo ATLAS (Router de modelos) ===\n")
    all_ok = True

    print("1. Grok (xAI)...")
    ok, msg = test_grok()
    status = "OK" if ok else "FAIL"
    print(f"   [{status}] {msg}\n")
    if not ok:
        all_ok = False

    print("2. DeepSeek...")
    ok, msg = test_deepseek()
    status = "OK" if ok else "FAIL"
    print(f"   [{status}] {msg}\n")
    if not ok:
        all_ok = False

    print("3. AWS Bedrock (Claude)...")
    ok, msg = test_bedrock()
    status = "OK" if ok else "FAIL"
    print(f"   [{status}] {msg}\n")
    if not ok:
        all_ok = False

    if all_ok:
        print("=== ATLAS operando al 100% con infraestructura privada. ===")
    else:
        print("=== Algunos proveedores fallaron. Revisa credenciales en la Bóveda.")
        print(
            "   Bedrock ValidationException: habilita el modelo en AWS Console (Bedrock > Model access). ==="
        )
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
