"""Coordinación de envío: dedup, rate limit por fingerprint, canales."""
from __future__ import annotations

import hashlib
import time
from typing import Any

from config.settings import settings as global_settings
from notifications.models import DispatchRecord
from operations.alert_dispatcher import get_alert_dispatcher


class OperationalNotificationDispatcher:
    def __init__(self, st: Any | None = None) -> None:
        self.settings = st or global_settings
        self._dedup_at: dict[str, float] = {}

    def _fingerprint(self, plain_body: str, kind: str) -> str:
        raw = f"{kind}|{plain_body[:4000]}".encode("utf-8", errors="replace")
        return hashlib.sha256(raw).hexdigest()[:20]

    def _is_duplicate(self, fp: str) -> bool:
        ttl = float(getattr(self.settings, "notify_dedup_ttl_sec", 600.0))
        now = time.time()
        cutoff = now - ttl
        self._dedup_at = {k: v for k, v in self._dedup_at.items() if v > cutoff}
        if fp in self._dedup_at:
            return True
        self._dedup_at[fp] = now
        return False

    def channel_set(self) -> set[str]:
        ch = list(getattr(self.settings, "notify_channels", ["telegram"]))
        return {c for c in ch if c in {"telegram", "whatsapp"}}

    async def send_briefing(
        self,
        *,
        kind: str,
        title: str,
        body_html: str,
        body_plain: str,
        category: str = "briefing_premarket",
        level: str | None = None,
    ) -> DispatchRecord:
        from operations.alert_dispatcher import LEVEL_INFO, LEVEL_WARNING

        fp = self._fingerprint(body_plain, kind)
        if self._is_duplicate(fp):
            return DispatchRecord(fingerprint=fp, channels=set(), ok=True, detail="deduplicated")
        ch = self.channel_set()
        if not ch:
            return DispatchRecord(fingerprint=fp, channels=set(), ok=False, detail="no_channels")
        disp = get_alert_dispatcher()
        lvl = level or (LEVEL_WARNING if "NO LISTO" in body_html or "NO LISTO" in body_plain else LEVEL_INFO)
        cd = float(getattr(self.settings, "notify_cooldown_sec", 180.0))
        await disp.operational_briefing(
            title=title,
            body_html=body_html,
            body_plain=body_plain,
            category=category,
            level=lvl,
            symbol="",
            metadata={"fingerprint": fp, "kind": kind},
            cooldown_s=cd,
            channels=ch,
        )
        return DispatchRecord(fingerprint=fp, channels=ch, ok=True, detail="sent")
