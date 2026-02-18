"""Llamadas a proveedores externos (Gemini, OpenAI, Anthropic) usando la clave del almacén. Solo httpx."""
from __future__ import annotations

import time
from typing import Optional, Tuple


def call_external(
    provider_id: str,
    model_name: str,
    prompt: str,
    system: Optional[str],
    api_key: str,
    timeout_s: int = 60,
) -> Tuple[bool, str, float]:
    """Llama al proveedor externo. Devuelve (ok, output_text, latency_ms)."""
    import httpx
    t0 = time.perf_counter()
    try:
        if provider_id == "gemini":
            return _call_gemini(model_name, prompt, system, api_key, timeout_s, t0)
        if provider_id == "openai":
            return _call_openai(model_name, prompt, system, api_key, timeout_s, t0)
        if provider_id == "anthropic":
            return _call_anthropic(model_name, prompt, system, api_key, timeout_s, t0)
        if provider_id == "perplexity":
            return _call_perplexity(model_name, prompt, system, api_key, timeout_s, t0)
        if provider_id == "groq":
            return _call_openai_compat("https://api.groq.com/openai/v1/chat/completions", model_name or "llama-3.3-70b-versatile", prompt, system, api_key, timeout_s, t0, "Groq")
        if provider_id == "xai":
            return _call_openai_compat("https://api.x.ai/v1/chat/completions", model_name or "grok-3-mini", prompt, system, api_key, timeout_s, t0, "xAI")
        if provider_id == "deepseek":
            return _call_openai_compat("https://api.deepseek.com/chat/completions", model_name or "deepseek-chat", prompt, system, api_key, timeout_s, t0, "DeepSeek")
        if provider_id == "mistral":
            return _call_openai_compat("https://api.mistral.ai/v1/chat/completions", model_name or "mistral-large-latest", prompt, system, api_key, timeout_s, t0, "Mistral")
        return False, "Proveedor no soportado: %s" % provider_id, (time.perf_counter() - t0) * 1000
    except Exception as e:
        return False, str(e), (time.perf_counter() - t0) * 1000


def _call_gemini(
    model: str, prompt: str, system: Optional[str], api_key: str, timeout_s: int, t0: float
) -> Tuple[bool, str, float]:
    model_id = (model or "gemini-1.5-flash").replace("/", "-").strip()
    url = "https://generativelanguage.googleapis.com/v1beta/models/%s:generateContent" % model_id
    params = {"key": api_key}
    text = (system.strip() + "\n\n" + prompt) if system and system.strip() else prompt
    body = {"contents": [{"parts": [{"text": text}]}], "generationConfig": {"temperature": 0.2}}
    with httpx.Client(timeout=timeout_s) as client:
        r = client.post(url, params=params, json=body)
    ms = (time.perf_counter() - t0) * 1000
    if r.status_code != 200:
        return False, "Gemini %s: %s" % (r.status_code, r.text[:300]), ms
    data = r.json()
    cand = (data.get("candidates") or [{}])[0]
    parts = (cand.get("content") or {}).get("parts") or []
    out = (parts[0].get("text") or "").strip() if parts else ""
    return True, out, ms


def _call_openai(
    model: str, prompt: str, system: Optional[str], api_key: str, timeout_s: int, t0: float
) -> Tuple[bool, str, float]:
    url = "https://api.openai.com/v1/chat/completions"
    headers = {"Authorization": "Bearer %s" % api_key, "Content-Type": "application/json"}
    messages = []
    if system and system.strip():
        messages.append({"role": "system", "content": system})
    messages.append({"role": "user", "content": prompt})
    body = {"model": model, "messages": messages, "temperature": 0.2}
    with httpx.Client(timeout=timeout_s) as client:
        r = client.post(url, headers=headers, json=body)
    ms = (time.perf_counter() - t0) * 1000
    if r.status_code != 200:
        return False, "OpenAI %s: %s" % (r.status_code, r.text[:300]), ms
    data = r.json()
    choice = (data.get("choices") or [{}])[0]
    out = (choice.get("message") or {}).get("content") or ""
    return True, out.strip(), ms


def _call_anthropic(
    model: str, prompt: str, system: Optional[str], api_key: str, timeout_s: int, t0: float
) -> Tuple[bool, str, float]:
    url = "https://api.anthropic.com/v1/messages"
    headers = {
        "x-api-key": api_key,
        "anthropic-version": "2023-06-01",
        "Content-Type": "application/json",
    }
    messages = [{"role": "user", "content": prompt}]
    body = {"model": model, "max_tokens": 4096, "messages": messages}
    if system and system.strip():
        body["system"] = system
    with httpx.Client(timeout=timeout_s) as client:
        r = client.post(url, headers=headers, json=body)
    ms = (time.perf_counter() - t0) * 1000
    if r.status_code != 200:
        return False, "Anthropic %s: %s" % (r.status_code, r.text[:300]), ms
    data = r.json()
    content = (data.get("content") or [{}])[0]
    out = (content.get("text") or "").strip()
    return True, out, ms


def _call_perplexity(
    model: str, prompt: str, system: Optional[str], api_key: str, timeout_s: int, t0: float
) -> Tuple[bool, str, float]:
    return _call_openai_compat("https://api.perplexity.ai/chat/completions", model or "sonar", prompt, system, api_key, timeout_s, t0, "Perplexity")


def _call_openai_compat(
    url: str, model: str, prompt: str, system: Optional[str], api_key: str, timeout_s: int, t0: float, label: str
) -> Tuple[bool, str, float]:
    """Generic caller for any OpenAI-compatible API (Groq, xAI/Grok, DeepSeek, Mistral, Perplexity)."""
    import httpx
    headers = {"Authorization": "Bearer %s" % api_key, "Content-Type": "application/json"}
    messages = []
    if system and system.strip():
        messages.append({"role": "system", "content": system})
    messages.append({"role": "user", "content": prompt})
    body = {"model": model, "messages": messages, "max_tokens": 4096, "temperature": 0.2}
    with httpx.Client(timeout=timeout_s) as client:
        r = client.post(url, headers=headers, json=body)
    ms = (time.perf_counter() - t0) * 1000
    if r.status_code != 200:
        return False, "%s %s: %s" % (label, r.status_code, r.text[:300]), ms
    data = r.json()
    choice = (data.get("choices") or [{}])[0]
    out = (choice.get("message") or {}).get("content") or ""
    return True, out.strip(), ms
