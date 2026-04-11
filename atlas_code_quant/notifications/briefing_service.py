"""Orquestación: contexto → priorización → render → persistencia → envío."""
from __future__ import annotations

import logging
from dataclasses import asdict
from datetime import datetime, timezone
from typing import Any

from config.settings import settings as global_settings
from notifications.dispatcher import OperationalNotificationDispatcher
from notifications.payloads import gather_operational_context
from notifications.prioritization import build_prioritized_eod, build_prioritized_premarket
from notifications.renderers import render_eod, render_exit_intelligence, render_intraday, render_premarket
from notifications.storage import NotificationAuditStore
from operations.alert_dispatcher import get_alert_dispatcher

logger = logging.getLogger("quant.notifications.briefing")


def _prioritized_as_dict(pb: Any) -> dict[str, Any]:
    d = asdict(pb)
    d["kind"] = pb.kind.value if hasattr(pb.kind, "value") else str(pb.kind)
    return d


_DEPS: dict[str, Any] | None = None
_SERVICE: "OperationalBriefingService | None" = None


def configure_operational_briefing(**kwargs: Any) -> None:
    global _DEPS, _SERVICE
    _DEPS = dict(kwargs)
    _SERVICE = None


def get_operational_briefing_service() -> "OperationalBriefingService":
    global _SERVICE
    if _SERVICE is None:
        if not _DEPS:
            raise RuntimeError("OperationalBriefingService: configure_operational_briefing() no llamado")
        _SERVICE = OperationalBriefingService(_DEPS)
    return _SERVICE


class OperationalBriefingService:
    def __init__(self, deps: dict[str, Any]) -> None:
        self._deps = deps
        self.settings = deps.get("settings") or global_settings
        self._dispatch = OperationalNotificationDispatcher(self.settings)
        self._store = NotificationAuditStore(self.settings.notifications_data_dir)

    def _ctx(self) -> dict[str, Any]:
        return gather_operational_context(
            operation_center=self._deps["operation_center"],
            scanner=self._deps["scanner"],
            canonical_service=self._deps["canonical_service"],
            vision_service=self._deps["vision_service"],
            st=self.settings,
            account_scope=self._deps.get("account_scope"),
        )

    def peek_context(self) -> dict[str, Any]:
        """Contexto semántico actual (para scheduler intradía sin enviar briefing completo)."""
        return self._ctx()

    def status(self) -> dict[str, Any]:
        try:
            dispatcher_status = get_alert_dispatcher().status()
        except Exception as exc:
            dispatcher_status = {"error": str(exc)}
        return {
            "notify_enabled": bool(getattr(self.settings, "notify_enabled", False)),
            "premarket": bool(getattr(self.settings, "notify_premarket", True)),
            "eod": bool(getattr(self.settings, "notify_eod", True)),
            "intraday": bool(getattr(self.settings, "notify_intraday", True)),
            "exit_intelligence": bool(getattr(self.settings, "notify_exit_intelligence", True)),
            "channels": list(getattr(self.settings, "notify_channels", [])),
            "dispatcher_status": dispatcher_status,
            "data_dir": str(self.settings.notifications_data_dir),
            "recent_snapshots": self._store.list_recent_snapshots(limit=10),
        }

    async def run_premarket(self) -> dict[str, Any]:
        ctx = self._ctx()
        pb = build_prioritized_premarket(ctx, self.settings)
        mode = str(getattr(self.settings, "notify_render_mode", "telegram_html"))
        html, plain = render_premarket(pb, mode=mode)
        rec = await self._dispatch.send_briefing(
            kind="premarket",
            title="ATLAS Quant — Preapertura",
            body_html=html,
            body_plain=plain,
            category="briefing_premarket",
        )
        payload = {
            "kind": "premarket",
            "generated_at": ctx.get("generated_at"),
            "prioritized": _prioritized_as_dict(pb),
            "context_summary": {
                "readiness_ready": bool((ctx.get("readiness") or {}).get("ready")),
                "open_positions": len((ctx.get("canonical_snapshot") or {}).get("positions") or []),
            },
            "dispatch": {"fingerprint": rec.fingerprint, "ok": rec.ok, "detail": rec.detail},
        }
        self._store.save_snapshot("premarket", payload)
        self._store.append_event(
            {
                "ts": datetime.now(timezone.utc).isoformat(),
                "kind": "premarket",
                "fingerprint": rec.fingerprint,
                "ok": rec.ok,
                "detail": rec.detail,
            }
        )
        return {"ok": True, "fingerprint": rec.fingerprint, "dedup": rec.detail == "deduplicated"}

    async def run_eod(self) -> dict[str, Any]:
        ctx = self._ctx()
        pb = build_prioritized_eod(ctx, self.settings)
        html, plain = render_eod(pb, mode=str(getattr(self.settings, "notify_render_mode", "telegram_html")))
        rec = await self._dispatch.send_briefing(
            kind="eod",
            title="ATLAS Quant — Cierre de sesión",
            body_html=html,
            body_plain=plain,
            category="briefing_eod",
        )
        payload = {
            "kind": "eod",
            "generated_at": ctx.get("generated_at"),
            "prioritized": _prioritized_as_dict(pb),
            "dispatch": {"fingerprint": rec.fingerprint, "ok": rec.ok, "detail": rec.detail},
        }
        self._store.save_snapshot("eod", payload)
        self._store.append_event(
            {
                "ts": datetime.now(timezone.utc).isoformat(),
                "kind": "eod",
                "fingerprint": rec.fingerprint,
                "ok": rec.ok,
            }
        )
        return {"ok": True, "fingerprint": rec.fingerprint}

    async def run_test(self, message: str) -> dict[str, Any]:
        title = "ATLAS Quant — Test briefing"
        html = f"<b>{title}</b>\n<code>{message}</code>"
        plain = f"{title}\n{message}"
        rec = await self._dispatch.send_briefing(
            kind="test",
            title=title,
            body_html=html,
            body_plain=plain,
            category="briefing_test",
        )
        return {"ok": rec.ok, "fingerprint": rec.fingerprint, "detail": rec.detail}

    async def emit_intraday(self, title: str, lines: list[str]) -> dict[str, Any]:
        html, plain = render_intraday(title, lines)
        rec = await self._dispatch.send_briefing(
            kind="intraday",
            title=title[:80],
            body_html=html,
            body_plain=plain,
            category="briefing_intraday",
        )
        self._store.append_event(
            {
                "ts": datetime.now(timezone.utc).isoformat(),
                "kind": "intraday",
                "title": title,
                "fingerprint": rec.fingerprint,
            }
        )
        return {"ok": rec.ok, "fingerprint": rec.fingerprint}

    async def emit_exit_intelligence(self, *, symbol: str, title_suffix: str, body_lines: list[str]) -> None:
        if not getattr(self.settings, "notify_enabled", False) or not getattr(
            self.settings, "notify_exit_intelligence", True
        ):
            return
        html, plain = render_exit_intelligence(symbol=symbol, title_suffix=title_suffix, body_lines=body_lines)
        await self._dispatch.send_briefing(
            kind="exit_intel",
            title=f"Salida — {symbol}",
            body_html=html,
            body_plain=plain,
            category="briefing_exit",
            level=None,
        )
        self._store.append_event(
            {
                "ts": datetime.now(timezone.utc).isoformat(),
                "kind": "exit_intelligence",
                "symbol": symbol,
            }
        )
