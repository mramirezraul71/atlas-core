"""LLM router: task -> Ollama model + system prompt. Config via .env."""
import os
from pathlib import Path

from dotenv import load_dotenv

# Cargar .env desde raíz del proyecto o config/
_repo = Path(__file__).resolve().parent.parent
for p in (_repo / ".env", _repo / "config" / ".env"):
    if p.exists():
        load_dotenv(p)
        break
else:
    load_dotenv()

from modules.providers.ollama_provider import ollama_chat

ATLAS_LLM_MODE = os.getenv("ATLAS_LLM_MODE", "local_first")
OLLAMA_BASE_URL = os.getenv("OLLAMA_BASE_URL", "http://127.0.0.1:11434")
OLLAMA_MODEL_FAST = os.getenv("OLLAMA_MODEL_FAST", "llama3.2:3b")
OLLAMA_MODEL_CHAT = os.getenv("OLLAMA_MODEL_CHAT", "llama3.1:latest")
OLLAMA_MODEL_CODE = os.getenv("OLLAMA_MODEL_CODE", "deepseek-coder:6.7b")
OLLAMA_MODEL_REASON = os.getenv("OLLAMA_MODEL_REASON", "deepseek-r1:14b")
OLLAMA_MODEL_TOOLS = os.getenv("OLLAMA_MODEL_TOOLS", "qwen2.5:7b")
LLM_TIMEOUT_SEC = int(os.getenv("LLM_TIMEOUT_SEC", "60"))

SYSTEM_PROMPTS = {
    "fast": "Responde de forma breve y accionable. Sin rodeos.",
    "chat": "Explica de forma clara y profesional.",
    "code": "Devuelve solo código, patch o comandos para Windows. Sin explicaciones largas.",
    "reason": "Da un plan paso a paso con chequeos. Sé conciso.",
    "tools": "Sugiere llamar al endpoint /execute de ATLAS y devuelve los pasos concretos (tool + args).",
}


def run(task: str, user_text: str, user: str = "raul") -> dict:
    """
    task in {"fast","chat","code","reason","tools"}.
    Returns dict: {ok, provider, model, output}.
    """
    task = (task or "chat").strip().lower()
    if task not in SYSTEM_PROMPTS:
        task = "chat"
    model_map = {
        "fast": OLLAMA_MODEL_FAST,
        "chat": OLLAMA_MODEL_CHAT,
        "code": OLLAMA_MODEL_CODE,
        "reason": OLLAMA_MODEL_REASON,
        "tools": OLLAMA_MODEL_TOOLS,
    }
    model = model_map[task]
    messages = [
        {"role": "system", "content": SYSTEM_PROMPTS[task]},
        {"role": "user", "content": user_text or ""},
    ]
    content = ollama_chat(model, messages, timeout_sec=LLM_TIMEOUT_SEC)
    return {
        "ok": True,
        "provider": "ollama",
        "model": model,
        "output": content,
    }
