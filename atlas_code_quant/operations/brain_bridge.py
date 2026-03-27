from __future__ import annotations

import json
import threading
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any
from urllib import error, parse, request

from config.settings import settings


_DEFAULT_STATE = {
    "enabled": True,
    "atlas_base_url": "",
    "memory_enabled": True,
    "bitacora_enabled": True,
    "last_delivery_ok": None,
    "last_bitacora_ok": None,
    "last_memory_ok": None,
    "last_event_kind": None,
    "last_event_at": None,
    "last_error": "",
    "total_events": 0,
    "delivered_events": 0,
    "queued_local_only_events": 0,
    "emitted_journal_keys": [],
}


class QuantBrainBridge:
    def __init__(
        self,
        *,
        base_url: str | None = None,
        api_key: str | None = None,
        state_path: Path | None = None,
        events_path: Path | None = None,
    ) -> None:
        self.base_url = (base_url or settings.atlas_brain_base_url or "").rstrip("/")
        self.api_key = api_key or settings.atlas_brain_api_key
        self.enabled = bool(settings.atlas_brain_enabled) and bool(self.base_url)
        self.memory_enabled = bool(settings.atlas_brain_memory_enabled)
        self.bitacora_enabled = bool(settings.atlas_brain_bitacora_enabled)
        self.source = settings.atlas_brain_source or "quant_brain"
        self.timeout_sec = max(int(settings.atlas_brain_timeout_sec or 5), 1)
        self.state_path = state_path or settings.atlas_brain_state_path
        self.events_path = events_path or settings.atlas_brain_events_path
        self._state_lock = threading.RLock()
        self._ensure_state()

    def _ensure_state(self) -> None:
        if not self.state_path.exists():
            payload = deepcopy(_DEFAULT_STATE)
            payload.update(
                {
                    "enabled": self.enabled,
                    "atlas_base_url": self.base_url,
                    "memory_enabled": self.memory_enabled,
                    "bitacora_enabled": self.bitacora_enabled,
                }
            )
            self._save_state(payload)

    def _load_state(self) -> dict[str, Any]:
        with self._state_lock:
            self._ensure_state()
            try:
                data = json.loads(self.state_path.read_text(encoding="utf-8"))
                if isinstance(data, dict):
                    merged = deepcopy(_DEFAULT_STATE)
                    merged.update(data)
                    merged["enabled"] = self.enabled
                    merged["atlas_base_url"] = self.base_url
                    merged["memory_enabled"] = self.memory_enabled
                    merged["bitacora_enabled"] = self.bitacora_enabled
                    merged["events_path"] = str(self.events_path)
                    return merged
            except Exception:
                pass
            fallback = deepcopy(_DEFAULT_STATE)
            fallback.update(
                {
                    "enabled": self.enabled,
                    "atlas_base_url": self.base_url,
                    "memory_enabled": self.memory_enabled,
                    "bitacora_enabled": self.bitacora_enabled,
                    "events_path": str(self.events_path),
                }
            )
            return fallback

    def _save_state(self, payload: dict[str, Any]) -> None:
        with self._state_lock:
            self.state_path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")

    def _append_event(self, record: dict[str, Any]) -> None:
        with self._state_lock:
            with self.events_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=True) + "\n")

    def _headers(self, content_type: str = "application/json") -> dict[str, str]:
        headers = {"Content-Type": content_type}
        if self.api_key:
            headers["X-Api-Key"] = self.api_key
        return headers

    def _post_json(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        req = request.Request(
            f"{self.base_url}{path}",
            data=json.dumps(payload).encode("utf-8"),
            headers=self._headers(),
            method="POST",
        )
        with request.urlopen(req, timeout=self.timeout_sec) as response:
            raw = response.read().decode("utf-8", errors="ignore")
        return json.loads(raw) if raw else {"ok": True}

    def _post_query(self, path: str, payload: dict[str, Any], *, timeout_sec: int | None = None) -> dict[str, Any]:
        query = parse.urlencode(payload, doseq=True)
        req = request.Request(
            f"{self.base_url}{path}?{query}",
            data=b"",
            headers=self._headers("application/x-www-form-urlencoded"),
            method="POST",
        )
        effective_timeout = max(int(timeout_sec or self.timeout_sec), 1)
        with request.urlopen(req, timeout=effective_timeout) as response:
            raw = response.read().decode("utf-8", errors="ignore")
        return json.loads(raw) if raw else {"ok": True}

    def status(self) -> dict[str, Any]:
        state = self._load_state()
        state["source"] = self.source
        state["events_path"] = str(self.events_path)
        return state

    def _journal_key_seen(self, journal_key: str) -> bool:
        state = self._load_state()
        emitted = state.get("emitted_journal_keys") or []
        return journal_key in emitted

    def _mark_journal_key(self, journal_key: str) -> None:
        with self._state_lock:
            state = self._load_state()
            emitted = list(state.get("emitted_journal_keys") or [])
            if journal_key not in emitted:
                emitted.append(journal_key)
            state["emitted_journal_keys"] = emitted[-500:]
            self._save_state(state)

    def emit(
        self,
        *,
        kind: str,
        message: str,
        level: str = "info",
        tags: list[str] | None = None,
        data: dict[str, Any] | None = None,
        description: str | None = None,
        context: str | None = None,
        outcome: str | None = None,
        memorize: bool = True,
    ) -> dict[str, Any]:
        now = datetime.utcnow().isoformat()
        record = {
            "timestamp": now,
            "kind": kind,
            "message": message,
            "level": level,
            "tags": tags or [],
            "data": data or {},
            "description": description or message,
            "context": context or "",
            "outcome": outcome or "",
            "source": self.source,
        }
        with self._state_lock:
            self._append_event(record)
            state = self._load_state()
            state["total_events"] = int(state.get("total_events") or 0) + 1
            state["last_event_kind"] = kind
            state["last_event_at"] = now

            delivered_ok = False
            bitacora_ok = None
            memory_ok = None
            errors: list[str] = []
            if self.enabled:
                if self.bitacora_enabled:
                    try:
                        self._post_json(
                            "/bitacora/log",
                            {
                                "message": message[:500],
                                "level": level,
                                "source": self.source,
                                "data": data or {},
                            },
                        )
                        bitacora_ok = True
                    except (error.URLError, error.HTTPError, TimeoutError, ValueError) as exc:
                        bitacora_ok = False
                        errors.append(f"bitacora: {exc}")
                if self.memory_enabled and memorize:
                    try:
                        self._post_query(
                            "/api/memory/add",
                            {
                                "description": (description or message)[:500],
                                "context": (context or "")[:2000],
                                "outcome": (outcome or "")[:500],
                                "tags": tags or [],
                            },
                            timeout_sec=max(self.timeout_sec, 15),
                        )
                        memory_ok = True
                    except (error.URLError, error.HTTPError, TimeoutError, ValueError) as exc:
                        memory_ok = False
                        errors.append(f"memory: {exc}")
                delivered_ok = bool(bitacora_ok) or bool(memory_ok)

            state["last_delivery_ok"] = delivered_ok
            state["last_bitacora_ok"] = bitacora_ok
            state["last_memory_ok"] = memory_ok
            state["last_error"] = " | ".join(errors)
            state["delivered_events"] = int(state.get("delivered_events") or 0) + (1 if delivered_ok else 0)
            state["queued_local_only_events"] = int(state.get("total_events") or 0) - int(state.get("delivered_events") or 0)
            self._save_state(state)
            return {"ok": delivered_ok, "record": record, "error": state["last_error"]}

    def record_selector_proposal(self, payload: dict[str, Any]) -> dict[str, Any]:
        candidate = payload.get("candidate") or {}
        selected = payload.get("selected") or {}
        risk_profile = payload.get("risk_profile") or {}
        confidence = payload.get("confidence_breakdown") or {}
        symbol = str(candidate.get("symbol") or "?")
        timeframe = str(candidate.get("timeframe") or "?")
        strategy_type = str(selected.get("strategy_type") or "?")
        message = (
            f"[QUANT][SELECTOR] {symbol} {timeframe} -> {strategy_type} | "
            f"score {selected.get('score') or '?'} | "
            f"win {confidence.get('probability_win_rate_pct') or confidence.get('local_win_rate_pct') or '?'}%"
        )
        return self.emit(
            kind="selector_proposal",
            level="info",
            message=message,
            tags=["quant", "selector", str(payload.get("account_scope") or "paper"), symbol, strategy_type],
            data={
                "candidate": candidate,
                "selected": selected,
                "risk_profile": risk_profile,
                "confidence_breakdown": confidence,
            },
            description=f"Quant selector propuso {strategy_type} para {symbol} en {timeframe}",
            context=json.dumps(
                {
                    "candidate": candidate,
                    "selected": selected,
                    "risk_profile": risk_profile,
                    "entry_plan": payload.get("entry_plan") or {},
                    "exit_plan": payload.get("exit_plan") or {},
                },
                ensure_ascii=False,
            ),
            outcome="proposal_generated",
        )

    def record_operation_cycle(self, payload: dict[str, Any], *, order: dict[str, Any] | None = None) -> dict[str, Any]:
        gates = payload.get("gates") or {}
        decision = str(payload.get("decision") or "unknown")
        action = str(payload.get("action") or "evaluate")
        symbol = str((order or {}).get("symbol") or "?")
        strategy_type = str((order or {}).get("strategy_type") or "?")
        allowed = bool(payload.get("allowed"))
        reasons = payload.get("reasons") or []
        level = "success" if allowed else "warning"
        message = (
            f"[QUANT][OPERACION] {decision} {action} {symbol} {strategy_type} | "
            f"scope {gates.get('scope') or '?'} | "
            f"win {gates.get('win_rate_pct') if gates.get('win_rate_pct') is not None else '?'}"
        )
        return self.emit(
            kind="operation_cycle",
            level=level,
            message=message,
            tags=[
                "quant",
                "operation",
                str(gates.get("scope") or "paper"),
                decision,
                action,
                symbol,
            ],
            data={
                "order": order or {},
                "payload": payload,
            },
            description=f"Quant {decision} {action} para {symbol}",
            context=json.dumps(
                {
                    "decision": decision,
                    "action": action,
                    "gates": gates,
                    "reasons": reasons,
                    "warnings": payload.get("warnings") or [],
                    "execution": payload.get("execution"),
                },
                ensure_ascii=False,
            ),
            outcome="allowed" if allowed else "blocked",
        )

    def record_error(self, *, kind: str, source: str, error_text: str, context: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.emit(
            kind=kind,
            level="error",
            message=f"[QUANT][ERROR] {source}: {error_text}",
            tags=["quant", "error", source],
            data=context or {},
            description=f"Error en {source}: {error_text}",
            context=json.dumps(context or {}, ensure_ascii=False),
            outcome="error",
            memorize=False,
        )

    def record_journal_outcome(self, entry: dict[str, Any]) -> dict[str, Any]:
        journal_key = str(entry.get("journal_key") or "")
        if not journal_key:
            return {"ok": False, "skipped": "missing_journal_key"}
        if self._journal_key_seen(journal_key):
            return {"ok": True, "skipped": "already_emitted", "journal_key": journal_key}

        realized_pnl = float(entry.get("realized_pnl") or 0.0)
        status = str(entry.get("status") or "")
        if status != "closed":
            return {"ok": True, "skipped": "not_closed", "journal_key": journal_key}

        if realized_pnl > 0:
            kind = "trade_victory"
            outcome = "win"
            level = "success"
        elif realized_pnl < 0:
            kind = "trade_error"
            outcome = "loss"
            level = "warning"
        else:
            kind = "trade_review"
            outcome = "flat"
            level = "info"

        symbol = str(entry.get("symbol") or "?")
        strategy_type = str(entry.get("strategy_type") or "?")
        post_mortem = entry.get("post_mortem") or {}
        message = (
            f"[QUANT][JOURNAL] {symbol} {strategy_type} cerrado | "
            f"resultado {realized_pnl:.2f} | causa {post_mortem.get('root_cause') or 'mixto'}"
        )
        result = self.emit(
            kind=kind,
            level=level,
            message=message,
            tags=[
                "quant",
                "journal",
                str(entry.get("account_type") or "paper"),
                symbol,
                strategy_type,
                outcome,
            ],
            data=entry,
            description=f"Cierre de {strategy_type} en {symbol} con resultado {realized_pnl:.2f}",
            context=json.dumps(
                {
                    "post_mortem": post_mortem,
                    "attribution": entry.get("attribution") or {},
                    "greeks": entry.get("greeks") or {},
                    "win_rate_at_entry": entry.get("win_rate_at_entry"),
                    "current_win_rate_pct": entry.get("current_win_rate_pct"),
                    "post_mortem_text": entry.get("post_mortem_text") or "",
                },
                ensure_ascii=False,
            ),
            outcome=outcome,
            memorize=True,
        )
        if result.get("ok"):
            self._mark_journal_key(journal_key)
        return result
