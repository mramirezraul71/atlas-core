"""
lotto_quant.llm.local_client
============================

Unified async client for **locally hosted** LLMs used by Atlas:
    - Ollama (default, http://localhost:11434)
    - OpenAI-compatible servers (vLLM, llama.cpp, LM Studio, text-generation-webui)

The Lotto-Quant module uses the LLM ONLY for:
    1. Natural-language explanation of EV signals  (Markov anomaly narration)
    2. Triage / sanity check of scraped raw HTML when parser fails
    3. Generation of human-readable Telegram alert text
    4. Optional reasoning over Kelly allocation borderline cases

It is NEVER used for the math itself — that lives in `models/`.
The LLM is strictly an explanation and routing layer, fully replaceable
with a local model so Atlas remains air-gapped if desired.

USAGE
-----
    from lotto_quant.llm import LocalAIClient
    async with LocalAIClient() as ai:
        text = await ai.complete("Explain this EV signal: ...")
"""

from __future__ import annotations

import asyncio
import json
import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import httpx

from .. import config

logger = logging.getLogger(__name__)


class LocalAIError(RuntimeError):
    """Raised when the local AI backend fails."""


@dataclass
class LocalAIResponse:
    text: str
    model: str
    backend: str
    raw: Dict[str, Any]


class LocalAIClient:
    """
    Async client for local LLM backends.

    Parameters
    ----------
    backend : str
        'ollama' or 'openai_compat'. Defaults to config.LOCAL_AI_BACKEND.
    base_url : str
        Server URL. Defaults to config.LOCAL_AI_BASE_URL.
    model : str
        Model identifier. Defaults to config.LOCAL_AI_MODEL.
    fallback_models : tuple
        Models to try if the primary returns 404 (model not pulled).
    """

    def __init__(
        self,
        backend: Optional[str] = None,
        base_url: Optional[str] = None,
        model: Optional[str] = None,
        fallback_models: Optional[Tuple[str, ...]] = None,
        timeout_s: Optional[int] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
    ):
        self.backend = (backend or config.LOCAL_AI_BACKEND).lower()
        self.base_url = (base_url or config.LOCAL_AI_BASE_URL).rstrip("/")
        self.model = model or config.LOCAL_AI_MODEL
        self.fallback_models = fallback_models or config.LOCAL_AI_FALLBACK_MODELS
        self.timeout_s = timeout_s or config.LOCAL_AI_TIMEOUT_S
        self.temperature = (
            temperature if temperature is not None else config.LOCAL_AI_TEMPERATURE
        )
        self.max_tokens = max_tokens or config.LOCAL_AI_MAX_TOKENS

        if self.backend not in ("ollama", "openai_compat"):
            raise ValueError(f"Unsupported backend: {self.backend}")

        self._client: Optional[httpx.AsyncClient] = None

    # ── async context management ────────────────────────────────────
    async def __aenter__(self) -> "LocalAIClient":
        self._client = httpx.AsyncClient(timeout=self.timeout_s)
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        if self._client is not None:
            await self._client.aclose()
            self._client = None

    # ── public API ──────────────────────────────────────────────────
    async def health(self) -> bool:
        """Return True if the backend responds to a simple ping."""
        if self._client is None:
            self._client = httpx.AsyncClient(timeout=10)
        try:
            if self.backend == "ollama":
                r = await self._client.get(f"{self.base_url}/api/tags")
            else:
                r = await self._client.get(f"{self.base_url}/v1/models")
            return r.status_code == 200
        except httpx.HTTPError as e:
            logger.warning("Local AI health check failed: %s", e)
            return False

    async def list_models(self) -> List[str]:
        """List models currently available on the local backend."""
        assert self._client is not None, "Use as async context manager"
        if self.backend == "ollama":
            r = await self._client.get(f"{self.base_url}/api/tags")
            r.raise_for_status()
            return [m["name"] for m in r.json().get("models", [])]
        r = await self._client.get(f"{self.base_url}/v1/models")
        r.raise_for_status()
        return [m["id"] for m in r.json().get("data", [])]

    async def complete(
        self,
        prompt: str,
        *,
        system: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        json_mode: bool = False,
    ) -> LocalAIResponse:
        """Single-turn completion with automatic fallback to other local models."""
        if self._client is None:
            self._client = httpx.AsyncClient(timeout=self.timeout_s)

        candidates = [self.model, *self.fallback_models]
        last_err: Optional[Exception] = None
        for model_name in candidates:
            try:
                return await self._complete_one(
                    model_name,
                    prompt=prompt,
                    system=system,
                    temperature=temperature,
                    max_tokens=max_tokens,
                    json_mode=json_mode,
                )
            except LocalAIError as e:
                logger.warning("Model %s failed: %s; trying next.", model_name, e)
                last_err = e
        raise LocalAIError(
            f"All local models failed (tried {candidates}). Last error: {last_err}"
        )

    async def complete_json(
        self,
        prompt: str,
        *,
        system: Optional[str] = None,
        schema_hint: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Force JSON output. Returns parsed dict; raises LocalAIError on parse failure."""
        json_system = (
            (system or "")
            + "\n\nIMPORTANT: Respond ONLY with a valid JSON object. "
            "No prose, no markdown fences, no commentary."
        )
        if schema_hint:
            json_system += f"\nExpected schema:\n{json.dumps(schema_hint, indent=2)}"
        resp = await self.complete(
            prompt,
            system=json_system.strip(),
            temperature=0.0,
            json_mode=True,
        )
        text = resp.text.strip()
        # tolerate ```json fences
        if text.startswith("```"):
            text = text.strip("`")
            if text.lower().startswith("json"):
                text = text[4:]
            text = text.strip()
        try:
            return json.loads(text)
        except json.JSONDecodeError as e:
            raise LocalAIError(f"Local model did not return valid JSON: {e}\n{text[:500]}")

    # ── internals ───────────────────────────────────────────────────
    async def _complete_one(
        self,
        model_name: str,
        *,
        prompt: str,
        system: Optional[str],
        temperature: Optional[float],
        max_tokens: Optional[int],
        json_mode: bool,
    ) -> LocalAIResponse:
        assert self._client is not None
        temp = temperature if temperature is not None else self.temperature
        max_t = max_tokens or self.max_tokens

        if self.backend == "ollama":
            payload: Dict[str, Any] = {
                "model": model_name,
                "prompt": prompt,
                "stream": False,
                "options": {
                    "temperature": temp,
                    "num_predict": max_t,
                },
            }
            if system:
                payload["system"] = system
            if json_mode:
                payload["format"] = "json"
            url = f"{self.base_url}/api/generate"
            try:
                r = await self._client.post(url, json=payload)
            except httpx.HTTPError as e:
                raise LocalAIError(f"Connection error to Ollama: {e}") from e
            if r.status_code == 404:
                raise LocalAIError(
                    f"Model '{model_name}' not found locally. "
                    f"Run: ollama pull {model_name}"
                )
            if r.status_code >= 400:
                raise LocalAIError(f"Ollama HTTP {r.status_code}: {r.text[:300]}")
            data = r.json()
            text = data.get("response", "")
            return LocalAIResponse(text=text, model=model_name, backend="ollama", raw=data)

        # OpenAI-compatible (vLLM, llama.cpp server, LM Studio, etc.)
        messages = []
        if system:
            messages.append({"role": "system", "content": system})
        messages.append({"role": "user", "content": prompt})
        payload = {
            "model": model_name,
            "messages": messages,
            "temperature": temp,
            "max_tokens": max_t,
        }
        if json_mode:
            payload["response_format"] = {"type": "json_object"}
        url = f"{self.base_url}/v1/chat/completions"
        try:
            r = await self._client.post(url, json=payload)
        except httpx.HTTPError as e:
            raise LocalAIError(f"Connection error to OpenAI-compat server: {e}") from e
        if r.status_code >= 400:
            raise LocalAIError(f"OpenAI-compat HTTP {r.status_code}: {r.text[:300]}")
        data = r.json()
        try:
            text = data["choices"][0]["message"]["content"]
        except (KeyError, IndexError) as e:
            raise LocalAIError(f"Malformed response: {e}\n{data}") from e
        return LocalAIResponse(text=text, model=model_name, backend="openai_compat", raw=data)


# ─────────────────────────────────────────────────────────────────────
# Sync convenience wrapper
# ─────────────────────────────────────────────────────────────────────
def complete_sync(prompt: str, **kwargs) -> str:
    """Run a single completion synchronously (useful in CLI / Streamlit)."""
    async def _run() -> str:
        async with LocalAIClient() as ai:
            r = await ai.complete(prompt, **kwargs)
            return r.text
    return asyncio.run(_run())
