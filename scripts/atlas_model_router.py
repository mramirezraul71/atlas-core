#!/usr/bin/env python3
"""
Router de modelos ATLAS: proxy local que lee credenciales y ejecuta llamadas con
fallback automático. Si una API falla o alcanza límite, salta al siguiente.
Bedrock se invoca vía boto3 (SDK AWS); el resto vía HTTP.
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent
VAULT = os.getenv("ATLAS_VAULT_PATH", r"C:\dev\credenciales.txt")
DEEPSEEK_ENDPOINT = os.getenv(
    "DEEPSEEK_ENDPOINT", "https://api.deepseek.com/v1/chat/completions"
)


def _load_vault() -> None:
    for p in [Path(VAULT), REPO_ROOT / ".env", REPO_ROOT / "config" / "atlas.env"]:
        if not p.is_file():
            continue
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
            break
        except Exception:
            continue


def _call_bedrock(
    prompt: str, system: Optional[str], model_id: str, timeout_s: int
) -> Tuple[bool, str, str]:
    """Claude en Bedrock vía boto3. model_id ej: us.anthropic.claude-opus-4-6-v1:0"""
    _load_vault()
    region = (
        os.getenv("AWS_REGION") or os.getenv("AWS_DEFAULT_REGION") or "us-east-1"
    ).strip()
    try:
        import boto3

        client = boto3.client("bedrock-runtime", region_name=region)
        body = {
            "anthropic_version": "bedrock-2023-05-31",
            "max_tokens": 4096,
            "messages": [{"role": "user", "content": prompt}],
        }
        if system and system.strip():
            body["system"] = system.strip()
        t0 = time.perf_counter()
        response = client.invoke_model(
            modelId=model_id,
            body=json.dumps(body),
        )
        data = json.loads(response["body"].read())
        text = (data.get("content") or [{}])[0].get("text", "")
        return True, text.strip(), "bedrock"
    except Exception as e:
        return False, str(e), "bedrock"


def _call_openai_compat(
    url: str,
    model: str,
    prompt: str,
    system: Optional[str],
    api_key: str,
    timeout_s: int,
    label: str,
) -> Tuple[bool, str, str]:
    import httpx

    headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
    messages = []
    if system and system.strip():
        messages.append({"role": "system", "content": system})
    messages.append({"role": "user", "content": prompt})
    body = {
        "model": model,
        "messages": messages,
        "max_tokens": 4096,
        "temperature": 0.2,
    }
    try:
        with httpx.Client(timeout=timeout_s) as client:
            r = client.post(url, headers=headers, json=body)
        if r.status_code != 200:
            return False, f"{label} {r.status_code}: {r.text[:400]}", label.lower()
        data = r.json()
        choice = (data.get("choices") or [{}])[0]
        out = (choice.get("message") or {}).get("content") or ""
        return True, out.strip(), label.lower()
    except Exception as e:
        return False, str(e), label.lower()


def _call_gemini(
    prompt: str, system: Optional[str], timeout_s: int
) -> Tuple[bool, str, str]:
    import httpx

    key = (os.getenv("GEMINI_API_KEY") or "").strip()
    if not key:
        return False, "no key", "gemini"
    url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:generateContent?key={key}"
    text = f"{system}\n\n{prompt}" if system and system.strip() else prompt
    body = {
        "contents": [{"parts": [{"text": text}]}],
        "generationConfig": {"maxOutputTokens": 4096},
    }
    try:
        with httpx.Client(timeout=timeout_s) as client:
            r = client.post(url, json=body)
        if r.status_code != 200:
            return False, f"Gemini {r.status_code}: {r.text[:200]}", "gemini"
        data = r.json()
        parts = (data.get("candidates") or [{}])[0].get("content", {}).get(
            "parts"
        ) or []
        out = (parts[0].get("text", "") if parts else "").strip()
        return True, out, "gemini"
    except Exception as e:
        return False, str(e), "gemini"


def complete(
    prompt: str,
    system: Optional[str] = None,
    route: str = "CHAT",
    timeout_s: int = 60,
) -> Dict[str, Any]:
    """
    Una sola completación con fallback en cadena.
    route: ARCHITECT -> prioriza Bedrock (Claude); REASON/CODE -> prioriza DeepSeek (barato).
    """
    _load_vault()

    if route.upper() == "ARCHITECT":
        chain: List[Tuple[str, Any]] = [
            (
                "bedrock",
                lambda: _call_bedrock(
                    prompt,
                    system,
                    os.getenv("ATLAS_MODEL_TUTOR", "us.anthropic.claude-opus-4-6-v1:0"),
                    timeout_s,
                ),
            ),
            (
                "deepseek",
                lambda: _call_openai_compat(
                    DEEPSEEK_ENDPOINT,
                    "deepseek-chat",
                    prompt,
                    system,
                    (os.getenv("DEEPSEEK_API_KEY") or "").strip(),
                    timeout_s,
                    "DeepSeek",
                )
                if (os.getenv("DEEPSEEK_API_KEY") or "").strip()
                else (False, "no key", "deepseek"),
            ),
            (
                "xai",
                lambda: _call_openai_compat(
                    "https://api.x.ai/v1/chat/completions",
                    "grok-3-mini",
                    prompt,
                    system,
                    (os.getenv("XAI_API_KEY") or "").strip(),
                    timeout_s,
                    "xAI",
                )
                if (os.getenv("XAI_API_KEY") or "").strip()
                else (False, "no key", "xai"),
            ),
            ("gemini", lambda: _call_gemini(prompt, system, timeout_s)),
        ]
    else:
        chain = [
            (
                "deepseek",
                lambda: _call_openai_compat(
                    DEEPSEEK_ENDPOINT,
                    "deepseek-chat",
                    prompt,
                    system,
                    (os.getenv("DEEPSEEK_API_KEY") or "").strip(),
                    timeout_s,
                    "DeepSeek",
                )
                if (os.getenv("DEEPSEEK_API_KEY") or "").strip()
                else (False, "no key", "deepseek"),
            ),
            (
                "bedrock",
                lambda: _call_bedrock(
                    prompt,
                    system,
                    os.getenv(
                        "ATLAS_MODEL_FAST",
                        "us.anthropic.claude-haiku-4-5-20251001-v1:0",
                    ),
                    timeout_s,
                ),
            ),
            (
                "xai",
                lambda: _call_openai_compat(
                    "https://api.x.ai/v1/chat/completions",
                    "grok-3-mini",
                    prompt,
                    system,
                    (os.getenv("XAI_API_KEY") or "").strip(),
                    timeout_s,
                    "xAI",
                )
                if (os.getenv("XAI_API_KEY") or "").strip()
                else (False, "no key", "xai"),
            ),
            (
                "groq",
                lambda: _call_openai_compat(
                    "https://api.groq.com/openai/v1/chat/completions",
                    "llama-3.3-70b-versatile",
                    prompt,
                    system,
                    (os.getenv("GROQ_API_KEY") or "").strip(),
                    timeout_s,
                    "Groq",
                )
                if (os.getenv("GROQ_API_KEY") or "").strip()
                else (False, "no key", "groq"),
            ),
            ("gemini", lambda: _call_gemini(prompt, system, timeout_s)),
        ]

    errors = []
    for name, fn in chain:
        try:
            ok, text, provider = fn()
            if ok and text:
                return {
                    "ok": True,
                    "text": text,
                    "provider": provider,
                    "errors": errors,
                }
            if not ok:
                errors.append(f"{provider}: {text[:200]}")
        except Exception as e:
            errors.append(f"{name}: {e}")
    return {"ok": False, "text": "", "provider": "", "errors": errors}


def main() -> int:
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--prompt", type=str, default="Di en una frase: ¿qué es 2+2?")
    p.add_argument("--route", type=str, default="CHAT")
    p.add_argument("--test-deepseek", action="store_true")
    p.add_argument("--test-grok", action="store_true")
    args = p.parse_args()

    _load_vault()

    if args.test_deepseek:
        key = (os.getenv("DEEPSEEK_API_KEY") or "").strip()
        if not key:
            print("DEEPSEEK: no key")
            return 1
        ok, text, _ = _call_openai_compat(
            DEEPSEEK_ENDPOINT,
            "deepseek-chat",
            "Responde solo: OK",
            None,
            key,
            30,
            "DeepSeek",
        )
        print("DEEPSEEK:", "OK" if ok else "FAIL", text[:200] if text else "")
        return 0 if ok else 1

    if args.test_grok:
        key = (os.getenv("XAI_API_KEY") or "").strip()
        if not key:
            print("GROK/XAI: no key")
            return 1
        ok, text, _ = _call_openai_compat(
            "https://api.x.ai/v1/chat/completions",
            "grok-3-mini",
            "Responde solo: OK",
            None,
            key,
            30,
            "xAI",
        )
        print("GROK:", "OK" if ok else "FAIL", text[:200] if text else "")
        return 0 if ok else 1

    out = complete(args.prompt, None, args.route)
    if out["ok"]:
        print(out["provider"], "=>", out["text"][:300])
    else:
        print("FAIL:", out["errors"])
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
