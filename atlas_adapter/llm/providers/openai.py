"""OpenAI Provider - Cloud LLM inference."""
import contextlib
import io
import os
from pathlib import Path
from typing import Any, Dict, Optional


class OpenAIProvider:
    """OpenAI cloud LLM provider with API key handling."""

    def __init__(self):
        self._load_vault_env()
        self.api_key = os.getenv("OPENAI_API_KEY", "")
        self.model = os.getenv("OPENAI_MODEL", "gpt-4o-mini")
        self.timeout = 30

    @staticmethod
    def _load_vault_env() -> None:
        """Best-effort: carga OPENAI_API_KEY desde la Bóveda si aún no está en entorno."""
        if (os.getenv("OPENAI_API_KEY") or "").strip():
            return
        try:
            from dotenv import load_dotenv
        except Exception:
            return

        candidates = [
            (os.getenv("ATLAS_VAULT_PATH") or "").strip(),
            r"C:\dev\credenciales.txt",
            r"C:\Users\Raul\OneDrive\RAUL - Personal\Escritorio\credenciales.txt",
        ]
        for candidate in candidates:
            if not candidate:
                continue
            try:
                path = Path(candidate)
                if path.is_file():
                    sink = io.StringIO()
                    with contextlib.redirect_stderr(sink), contextlib.redirect_stdout(sink):
                        load_dotenv(str(path), override=True)
                    return
            except Exception:
                continue

    def is_available(self) -> bool:
        """Check if OpenAI API key is configured."""
        return bool((self.api_key or "").strip())

    def generate(
        self, prompt: str, context: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Generate completion using OpenAI API.

        Args:
            prompt: The prompt to send to the model
            context: Optional context dictionary

        Returns:
            Dict with 'ok', 'response', 'provider', 'error' fields
        """
        if not self.is_available():
            return {
                "ok": False,
                "error": "OpenAI API key not configured",
                "provider": "openai",
                "response": None,
            }

        try:
            from openai import OpenAI

            client = OpenAI(api_key=self.api_key, timeout=self.timeout)

            messages = [{"role": "user", "content": prompt}]

            response = client.chat.completions.create(
                model=self.model, messages=messages, temperature=0.7, max_tokens=1000
            )

            content = response.choices[0].message.content

            return {
                "ok": True,
                "response": content,
                "provider": "openai",
                "model": self.model,
                "error": None,
            }

        except Exception as e:
            return {
                "ok": False,
                "error": str(e),
                "provider": "openai",
                "response": None,
            }
