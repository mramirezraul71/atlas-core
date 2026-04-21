from __future__ import annotations

import json
import os
import random
import re
import socket
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, TypedDict


GROK_SYSTEM_PROMPT = (
    "You are ATLAS Options Expert Reviewer. Review paper-only options trades in advisory mode. "
    "You do not control execution. Return only compact JSON with keys: "
    "verdict, score_adjustment, contracts_multiplier, prefer_strategy, rationale, confidence. "
    "Use verdict in {'approve','reject','neutral'}. "
    "Use score_adjustment as a small float, contracts_multiplier in [0,1], "
    "prefer_strategy as null or an uppercase strategy identifier. "
    "If the setup is acceptable but not strong, use neutral instead of reject."
)


def parse_env_flag(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    value = raw.strip().lower()
    if value in {"1", "true", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "no", "n", "off"}:
        return False
    return default


class TransientGrokError(RuntimeError):
    """Error transitorio: timeout, 429, 5xx o fallo de red recuperable."""


class GrokReviewResult(TypedDict):
    provider: str
    status: str
    verdict: str
    score_adjustment: float | None
    contracts_multiplier: float | None
    prefer_strategy: str | None
    confidence: float | None
    rationale: str
    error: str | None


@dataclass(slots=True)
class GrokReviewConfig:
    api_key_env: str = "XAI_API_KEY"
    api_url: str = field(default_factory=lambda: os.getenv("XAI_API_URL", "https://api.x.ai/v1/chat/completions"))
    model: str = field(default_factory=lambda: os.getenv("XAI_MODEL", "grok-2-latest"))
    timeout_seconds: float = field(default_factory=lambda: float(os.getenv("XAI_TIMEOUT_SECONDS", "8")))
    temperature: float = field(default_factory=lambda: float(os.getenv("XAI_TEMPERATURE", "0.1")))
    max_tokens: int = field(default_factory=lambda: int(os.getenv("XAI_MAX_TOKENS", "400")))
    max_retries: int = field(default_factory=lambda: int(os.getenv("XAI_MAX_RETRIES", "2")))
    backoff_base_seconds: float = field(default_factory=lambda: float(os.getenv("XAI_BACKOFF_BASE_SECONDS", "0.35")))
    enabled: bool = field(default_factory=lambda: parse_env_flag("QUANT_GROK_REVIEW_ENABLED", False))

    @property
    def api_key(self) -> str | None:
        raw = os.getenv(self.api_key_env, "").strip()
        return raw or None


class GrokExpertReviewProvider:
    """Thin xAI adapter for advisory paper-only options reviews.

    The provider is intentionally side-effect free: it accepts a JSON-safe
    decision pack and returns a normalized review or a neutral fallback.
    """

    def __init__(self, config: GrokReviewConfig | None = None) -> None:
        self.config = config or GrokReviewConfig()

    def review_trade(self, decision_pack: dict[str, Any]) -> GrokReviewResult:
        if not self.config.enabled:
            return self._safe_fallback("grok_review_disabled")
        if not self.config.api_key:
            return self._safe_fallback("missing_xai_api_key")
        try:
            raw_text = self._request_review_text(decision_pack)
        except Exception as exc:
            return self._safe_fallback(f"api_error:{exc}")

        parsed = self._parse_json_payload(raw_text)
        if parsed is None:
            return self._safe_fallback("parse_error")
        return self._normalize_review(parsed)

    def _request_review_text(self, decision_pack: dict[str, Any]) -> str:
        attempts = max(0, int(self.config.max_retries)) + 1
        last_error: Exception | None = None
        for attempt in range(attempts):
            try:
                return self._request_review_text_once(decision_pack)
            except TransientGrokError as exc:
                last_error = exc
                if attempt >= attempts - 1:
                    break
                self._sleep(self._compute_backoff_delay(attempt))
            except Exception:
                raise
        raise RuntimeError(f"transient_retries_exhausted:{last_error}")

    def _request_review_text_once(self, decision_pack: dict[str, Any]) -> str:
        body = {
            "model": self.config.model,
            "temperature": self.config.temperature,
            "max_tokens": self.config.max_tokens,
            "response_format": {"type": "json_object"},
            "messages": [
                {"role": "system", "content": GROK_SYSTEM_PROMPT},
                {
                    "role": "user",
                    "content": json.dumps(decision_pack, ensure_ascii=True, separators=(",", ":")),
                },
            ],
        }
        req = urllib.request.Request(
            self.config.api_url,
            data=json.dumps(body).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.config.api_key}",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.config.timeout_seconds) as response:
                payload = json.loads(response.read().decode("utf-8", errors="replace"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace") if hasattr(exc, "read") else str(exc)
            if exc.code == 429 or 500 <= exc.code <= 599:
                raise TransientGrokError(f"http_{exc.code}:{detail[:180]}") from exc
            raise RuntimeError(f"http_{exc.code}:{detail[:180]}") from exc
        except (urllib.error.URLError, TimeoutError, socket.timeout) as exc:
            raise TransientGrokError(f"network_error:{exc}") from exc
        return self._extract_text_from_response(payload)

    def _extract_text_from_response(self, payload: dict[str, Any]) -> str:
        choices = payload.get("choices") or []
        if not choices:
            return json.dumps(payload)
        message = (choices[0] or {}).get("message") or {}
        content = message.get("content")
        if isinstance(content, str):
            return content
        if isinstance(content, list):
            text_chunks: list[str] = []
            for item in content:
                if isinstance(item, dict):
                    text_val = item.get("text")
                    if isinstance(text_val, str):
                        text_chunks.append(text_val)
            if text_chunks:
                return "".join(text_chunks)
        return json.dumps(message or payload)

    def _parse_json_payload(self, raw_text: str) -> dict[str, Any] | None:
        raw_text = (raw_text or "").strip()
        if not raw_text:
            return None
        try:
            parsed = json.loads(raw_text)
            return parsed if isinstance(parsed, dict) else None
        except json.JSONDecodeError:
            pass

        fenced = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", raw_text, flags=re.DOTALL | re.IGNORECASE)
        if fenced:
            try:
                parsed = json.loads(fenced.group(1))
                return parsed if isinstance(parsed, dict) else None
            except json.JSONDecodeError:
                return None

        start = raw_text.find("{")
        end = raw_text.rfind("}")
        if start == -1 or end == -1 or end <= start:
            return None
        try:
            parsed = json.loads(raw_text[start : end + 1])
            return parsed if isinstance(parsed, dict) else None
        except json.JSONDecodeError:
            return None

    def _normalize_review(self, parsed: dict[str, Any]) -> GrokReviewResult:
        verdict_raw = str(parsed.get("verdict") or "neutral").strip().lower()
        verdict = verdict_raw if verdict_raw in {"approve", "reject", "neutral"} else "neutral"
        score_adjustment = self._safe_float(parsed.get("score_adjustment"))
        contracts_multiplier = self._safe_float(parsed.get("contracts_multiplier"))
        if contracts_multiplier is not None:
            contracts_multiplier = max(0.0, min(1.0, contracts_multiplier))
        prefer_strategy = parsed.get("prefer_strategy")
        if prefer_strategy is not None:
            prefer_strategy = str(prefer_strategy).strip().upper() or None
        confidence = self._safe_float(parsed.get("confidence"))
        if confidence is not None:
            confidence = max(0.0, min(1.0, confidence))
        rationale = str(parsed.get("rationale") or "grok_review_completed").strip()
        return {
            "provider": "grok",
            "status": "ok",
            "verdict": verdict,
            "score_adjustment": score_adjustment,
            "contracts_multiplier": contracts_multiplier,
            "prefer_strategy": prefer_strategy,
            "confidence": confidence,
            "rationale": rationale,
            "error": None,
        }

    def _compute_backoff_delay(self, attempt: int) -> float:
        base = max(0.05, float(self.config.backoff_base_seconds))
        jitter = random.uniform(0.0, base * 0.25)
        return (base * (2**attempt)) + jitter

    def _sleep(self, seconds: float) -> None:
        time.sleep(max(0.0, seconds))

    def _safe_fallback(self, reason: str) -> GrokReviewResult:
        return {
            "provider": "grok",
            "status": "fallback",
            "verdict": "neutral",
            "score_adjustment": 0.0,
            "contracts_multiplier": None,
            "prefer_strategy": None,
            "confidence": None,
            "rationale": "grok_unavailable_pipeline_continues",
            "error": reason,
        }

    @staticmethod
    def _safe_float(value: Any) -> float | None:
        try:
            if value is None or value == "":
                return None
            return float(value)
        except (TypeError, ValueError):
            return None
