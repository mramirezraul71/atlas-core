from __future__ import annotations
import os

def _env(name: str, default: str) -> str:
    v = os.getenv(name)
    return v.strip() if v and v.strip() else default

def _env_int(name: str, default: int) -> int:
    try:
        return int(_env(name, str(default)))
    except Exception:
        return default

def _env_float(name: str, default: float) -> float:
    try:
        return float(_env(name, str(default)))
    except Exception:
        return default

def _env_bool(name: str, default: bool) -> bool:
    v = _env(name, "true" if default else "false").lower()
    return v in ("1", "true", "yes", "y", "on")

class LLMSettings:
    OLLAMA_BASE_URL = _env("OLLAMA_BASE_URL", "http://127.0.0.1:11434")

    MODEL_FAST   = _env("LLM_MODEL_FAST", "llama3.2:3b")
    MODEL_CHAT   = _env("LLM_MODEL_CHAT", "llama3.1:latest")
    MODEL_CODE   = _env("LLM_MODEL_CODE", "deepseek-coder:6.7b")
    MODEL_REASON = _env("LLM_MODEL_REASON", "deepseek-r1:14b")
    MODEL_TOOLS  = _env("LLM_MODEL_TOOLS", "qwen2.5:7b")

    FAST_MAX_CHARS   = _env_int("LLM_FAST_MAX_CHARS", 900)
    CHAT_MAX_CHARS   = _env_int("LLM_CHAT_MAX_CHARS", 6000)
    REASON_MIN_CHARS = _env_int("LLM_REASON_MIN_CHARS", 1800)

    TIMEOUT_CONNECT = _env_int("LLM_TIMEOUT_CONNECT", 5)
    TIMEOUT_READ    = _env_int("LLM_TIMEOUT_READ", 120)

    DEFAULT_ROUTE = _env("LLM_DEFAULT_ROUTE", "CHAT").upper()
    ENABLE_AUDIT = _env_bool("LLM_ENABLE_AUDIT", True)

    def model_for_route(self, route: str) -> str:
        r = (route or self.DEFAULT_ROUTE).upper()
        if r == "FAST": return self.MODEL_FAST
        if r == "CHAT": return self.MODEL_CHAT
        if r == "CODE": return self.MODEL_CODE
        if r == "REASON": return self.MODEL_REASON
        if r == "TOOLS": return self.MODEL_TOOLS
        return self.MODEL_CHAT
