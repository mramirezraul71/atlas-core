"""
Local-AI subsystem for Atlas Lotto-Quant.

Replaces cloud LLMs with a locally-hosted backend (Ollama / DeepSeek / Qwen3).
Single entry point: `LocalAIClient` from `local_client.py`.
"""

from .local_client import LocalAIClient, LocalAIError, LocalAIResponse

__all__ = ["LocalAIClient", "LocalAIError", "LocalAIResponse"]
