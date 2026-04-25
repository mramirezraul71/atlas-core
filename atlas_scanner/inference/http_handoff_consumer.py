from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import logging
import os
import time
from typing import Any

import requests

from atlas_scanner.contracts import RadarDecisionHandoff

logger = logging.getLogger("atlas_scanner.handoff.http")


@dataclass(frozen=True)
class HttpHandoffConfig:
    endpoint_url: str
    timeout_sec: float = 5.0
    retries: int = 2
    backoff_sec: float = 0.5
    auth_bearer_token: str | None = None


@dataclass
class HttpHandoffConsumer:
    config: HttpHandoffConfig

    def consume(self, handoff: RadarDecisionHandoff) -> None:
        payload = _handoff_to_payload(handoff)
        body = json.dumps(payload, ensure_ascii=False)
        idempotency_key = _idempotency_key(body)
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
            "Idempotency-Key": idempotency_key,
            "X-Atlas-Radar-Handoff-Id": idempotency_key,
        }
        if self.config.auth_bearer_token:
            headers["Authorization"] = f"Bearer {self.config.auth_bearer_token}"
        last_error: str | None = None
        for attempt in range(self.config.retries + 1):
            try:
                response = requests.post(
                    self.config.endpoint_url,
                    data=body.encode("utf-8"),
                    headers=headers,
                    timeout=self.config.timeout_sec,
                )
                if response.status_code < 400:
                    logger.info(
                        "handoff http published endpoint=%s status=%s attempt=%s idempotency_key=%s",
                        self.config.endpoint_url,
                        response.status_code,
                        attempt,
                        idempotency_key,
                    )
                    return
                last_error = f"http_status_{response.status_code}"
            except Exception as exc:
                last_error = str(exc)
            if attempt < self.config.retries:
                time.sleep(self.config.backoff_sec * (attempt + 1))
        logger.warning(
            "handoff http failed endpoint=%s retries=%s error=%s idempotency_key=%s",
            self.config.endpoint_url,
            self.config.retries,
            last_error or "unknown",
            idempotency_key,
        )


def config_from_env() -> HttpHandoffConfig | None:
    enabled = os.getenv("ATLAS_RADAR_HANDOFF_HTTP_ENABLED", "false").strip().lower() in {"1", "true", "yes"}
    endpoint = os.getenv("ATLAS_RADAR_HANDOFF_HTTP_URL", "").strip()
    if not enabled or not endpoint:
        return None
    timeout_sec = float(os.getenv("ATLAS_RADAR_HANDOFF_HTTP_TIMEOUT_SEC", "5.0"))
    retries = int(os.getenv("ATLAS_RADAR_HANDOFF_HTTP_RETRIES", "2"))
    backoff_sec = float(os.getenv("ATLAS_RADAR_HANDOFF_HTTP_BACKOFF_SEC", "0.5"))
    token = os.getenv("ATLAS_RADAR_HANDOFF_HTTP_BEARER_TOKEN", "").strip() or None
    return HttpHandoffConfig(
        endpoint_url=endpoint,
        timeout_sec=timeout_sec,
        retries=max(0, retries),
        backoff_sec=max(0.0, backoff_sec),
        auth_bearer_token=token,
    )


def _idempotency_key(body: str) -> str:
    return hashlib.sha256(body.encode("utf-8")).hexdigest()


def _handoff_to_payload(handoff: RadarDecisionHandoff) -> dict[str, Any]:
    return {
        "symbol": handoff.symbol,
        "as_of": handoff.as_of.isoformat(),
        "operable": handoff.operable,
        "primary_timeframe": handoff.primary_timeframe,
        "handoff_summary": handoff.handoff_summary,
        "degradation_reasons": list(handoff.degradation_reasons),
        "metadata": dict(handoff.metadata),
        "signals": [
            {
                "timeframe": signal.timeframe,
                "aggregate_conviction_score": signal.aggregate_conviction_score,
                "primary_conviction_reason": signal.primary_conviction_reason,
                "primary_degradation_reason": signal.primary_degradation_reason,
                "operable": signal.quality.is_operable,
            }
            for signal in handoff.signals
        ],
    }
