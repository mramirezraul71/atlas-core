"""Cliente de intake Radar (REST polling) para cutover F3."""
from __future__ import annotations

import json
import logging
import socket
from dataclasses import dataclass, field
from typing import Any
from urllib import error as urllib_error
from urllib import parse as urllib_parse
from urllib import request as urllib_request

from atlas_code_quant.intake.opportunity import RadarOpportunity, RadarOpportunityBatch

logger = logging.getLogger("quant.intake.radar")

@dataclass(slots=True)
class RadarClientResult:
    """Resultado auditable de consumo Radar."""

    ok: bool
    status_code: int
    batch: RadarOpportunityBatch
    error: str | None = None
    trace_id: str = ""
    degraded: bool = False


@dataclass(slots=True)
class RadarOpportunityClient:
    """Cliente HTTP conservador para ``/api/radar/opportunities``."""

    opportunities_url: str
    stream_url: str = ""
    enabled: bool = True
    timeout_sec: float = 4.0
    default_limit: int = 24
    default_min_score: float = 80.0
    user_agent: str = "atlas-codequant-radar-intake/1.0"
    _last_trace_id: str = field(default="", init=False)

    @property
    def last_trace_id(self) -> str:
        return self._last_trace_id

    def _request_json(self, url: str) -> tuple[int, dict[str, Any], dict[str, str]]:
        req = urllib_request.Request(
            url,
            headers={
                "Accept": "application/json",
                "User-Agent": self.user_agent,
                "Cache-Control": "no-cache",
            },
            method="GET",
        )
        with urllib_request.urlopen(req, timeout=float(self.timeout_sec)) as resp:
            code = int(getattr(resp, "status", 200) or 200)
            headers = {k.lower(): v for k, v in dict(resp.headers or {}).items()}
            body = resp.read().decode("utf-8", errors="replace")
        parsed = json.loads(body) if body else {}
        if not isinstance(parsed, dict):
            raise ValueError("Respuesta Radar no es objeto JSON")
        return code, parsed, headers

    def fetch_opportunities(
        self,
        *,
        limit: int | None = None,
        min_score: float | None = None,
        asset_class: str | None = None,
        sector: str | None = None,
    ) -> RadarClientResult:
        if not self.enabled:
            return RadarClientResult(
                ok=False,
                status_code=503,
                degraded=True,
                error="radar_intake_disabled",
                trace_id="",
                batch=RadarOpportunityBatch(
                    ok=False,
                    message="Radar intake deshabilitado por flag.",
                    source="disabled",
                ),
            )
        lim = max(1, int(limit if limit is not None else self.default_limit))
        ms = float(min_score if min_score is not None else self.default_min_score)
        q: dict[str, str] = {"limit": str(lim), "min_score": str(ms)}
        if asset_class:
            q["asset_class"] = str(asset_class).strip()
        if sector:
            q["sector"] = str(sector).strip()
        url = f"{self.opportunities_url}?{urllib_parse.urlencode(q)}"
        try:
            code, payload, _headers = self._request_json(url)
            batch = RadarOpportunityBatch.from_response(payload)
            trace_id = batch.trace_id or ""
            self._last_trace_id = trace_id
            if code >= 400:
                msg = str(payload.get("message") or payload.get("detail") or f"radar_http_{code}")
                return RadarClientResult(
                    ok=False,
                    status_code=code,
                    error=msg,
                    degraded=True,
                    trace_id=trace_id,
                    batch=batch,
                )
            return RadarClientResult(
                ok=True,
                status_code=code,
                error=None,
                degraded=False,
                trace_id=trace_id,
                batch=batch,
            )
        except urllib_error.HTTPError as exc:
            body = ""
            try:
                body = exc.read().decode("utf-8", errors="replace")
            except Exception:
                body = ""
            msg = f"radar_http_error_{exc.code}"
            if body:
                msg = f"{msg}:{body[:220]}"
            logger.warning("radar_client.fetch_opportunities http_error code=%s msg=%s", exc.code, msg)
            return RadarClientResult(
                ok=False,
                status_code=int(exc.code),
                error=msg,
                degraded=True,
                trace_id=self._last_trace_id,
                batch=RadarOpportunityBatch(ok=False, message=msg, source="error"),
            )
        except (urllib_error.URLError, TimeoutError, socket.timeout) as exc:
            msg = f"radar_unreachable:{exc.__class__.__name__}"
            logger.warning("radar_client.fetch_opportunities transport_error=%s", msg)
            return RadarClientResult(
                ok=False,
                status_code=503,
                error=msg,
                degraded=True,
                trace_id=self._last_trace_id,
                batch=RadarOpportunityBatch(ok=False, message=msg, source="unreachable"),
            )
        except Exception as exc:
            msg = f"radar_parse_error:{exc.__class__.__name__}"
            logger.exception("radar_client.fetch_opportunities parse_error=%s", msg)
            return RadarClientResult(
                ok=False,
                status_code=500,
                error=msg,
                degraded=True,
                trace_id=self._last_trace_id,
                batch=RadarOpportunityBatch(ok=False, message=msg, source="error"),
            )

    def get_opportunity(self, symbol: str, *, min_score: float | None = None) -> RadarOpportunity | None:
        sym = str(symbol or "").strip().upper()
        if not sym:
            return None
        ms = float(min_score if min_score is not None else self.default_min_score)
        q = urllib_parse.urlencode({"min_score": str(ms)})
        url = f"{self.opportunities_url.rstrip('/')}/{sym}?{q}"
        try:
            code, payload, _headers = self._request_json(url)
            if code >= 400:
                logger.info("radar_client.get_opportunity symbol=%s status=%s", sym, code)
                return None
            raw = payload.get("opportunity")
            if not isinstance(raw, dict):
                return None
            opp = RadarOpportunity.from_dict(raw)
            self._last_trace_id = str(raw.get("trace_id") or payload.get("trace_id") or self._last_trace_id)
            return opp
        except Exception as exc:
            logger.warning("radar_client.get_opportunity symbol=%s error=%s", sym, exc.__class__.__name__)
            return None

    def stream_opportunities(self) -> list[dict[str, Any]]:
        """F3 usa polling REST. Stream SSE queda reservado para fase posterior."""
        logger.info("radar_client.stream_opportunities not_implemented stream_url=%s", self.stream_url)
        return []
