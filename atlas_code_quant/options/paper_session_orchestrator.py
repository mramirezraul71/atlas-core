"""Orquestador paper-only: briefing → intent → plan de entrada (sin órdenes)."""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from atlas_code_quant.config.settings import get_options_engine_market_open_runtime, settings
from atlas_code_quant.learning.trading_self_audit_protocol import read_trading_self_audit_protocol
from atlas_code_quant.operations.operational_self_audit import (
    OperationalSelfAuditContext,
    operational_self_audit_enabled,
    run_operational_self_audit,
)
from atlas_code_quant.options.options_paper_journal import OptionsPaperJournal
from atlas_code_quant.options.paper_entry_planner import PaperEntryPlanner
from atlas_code_quant.options.options_intent_router import (
    KNOWN_STRATEGY_FAMILIES,
    OptionsIntentRouter,
)
from atlas_code_quant.atlas_core_provenance import attach_atlas_core_provenance_to_plan
from atlas_code_quant.options import options_engine_metrics as _options_metrics
from atlas_code_quant.options.session_briefing import SessionBriefingEngine
from atlas_code_quant.signals.visual_signal_adapter import VisualSignalAdapter


def _options_repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _attach_options_self_audit(data: dict[str, Any]) -> None:
    """Self-audit operativo ligero: observacional, no bloquea el ciclo paper."""
    ts = datetime.now(timezone.utc).isoformat()
    if not operational_self_audit_enabled(settings):
        data["options_self_audit"] = {
            "status": "skipped",
            "source": "operational_self_audit",
            "timestamp": ts,
            "findings_count": 0,
            "blocking_findings": 0,
            "passed": None,
            "overall_severity": None,
            "reason": "audit_disabled_or_scope",
        }
        return

    mor = data.get("market_open_runtime") if isinstance(data.get("market_open_runtime"), dict) else {}
    max_open = mor.get("max_open_positions")
    market_snap: dict[str, Any] = {}
    if max_open is not None:
        try:
            market_snap["max_positions"] = int(max_open)
        except (TypeError, ValueError):
            market_snap["max_positions"] = max_open

    protocol: dict[str, Any] = {}
    proto_path = _options_repo_root() / "reports" / "trading_self_audit_protocol.json"
    try:
        protocol = read_trading_self_audit_protocol(proto_path)
    except Exception:
        protocol = {}

    try:
        ctx = OperationalSelfAuditContext(
            settings=settings,
            operation_config={},
            market_open_snapshot=market_snap,
            protocol=protocol,
            positions_summary=None,
            scope="paper",
        )
        result = run_operational_self_audit(ctx)
        blocking = sum(1 for c in result.checks if getattr(c, "severity", None) == "BLOCK")
        data["options_self_audit"] = {
            "status": "ok",
            "source": "operational_self_audit",
            "timestamp": result.ts_utc or ts,
            "passed": result.passed,
            "overall_severity": result.overall_severity,
            "findings_count": len(result.checks),
            "blocking_findings": blocking,
        }
    except Exception as exc:
        _options_metrics.record_options_error("options_self_audit_exception")
        data["options_self_audit"] = {
            "status": "error",
            "source": "operational_self_audit",
            "timestamp": ts,
            "findings_count": 0,
            "blocking_findings": 0,
            "passed": None,
            "overall_severity": None,
            "error": f"{type(exc).__name__}: {exc}",
        }


def _optional_positive_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if v > 0 else None


def _merge_breach_contexts(caller: dict[str, Any] | None, adapter: dict[str, Any]) -> dict[str, Any]:
    """Combina breach del caller con salida ``VisualSignalAdapter`` (OR en flags; ``details`` del adaptador)."""
    cb = dict(caller or {})
    ad = dict(adapter or {})
    merged = dict(ad)
    merged["breach_detected"] = bool(cb.get("breach_detected")) or bool(ad.get("breach_detected"))
    merged["critical_breach"] = bool(cb.get("critical_breach")) or bool(ad.get("critical_breach"))
    merged["short_strike_breached"] = bool(cb.get("short_strike_breached")) or bool(ad.get("short_strike_breached"))
    merged["risk_break"] = bool(cb.get("risk_break")) or bool(ad.get("risk_break"))
    if ad.get("breach_detected") and ad.get("level_broken") is not None:
        merged["level_broken"] = ad.get("level_broken")
    elif cb.get("breach_detected") and cb.get("level_broken") is not None:
        merged["level_broken"] = cb.get("level_broken")
    else:
        merged["level_broken"] = ad.get("level_broken") if ad.get("level_broken") is not None else cb.get("level_broken")
    merged["details"] = dict(ad.get("details") or {})
    return merged


class PaperSessionOrchestrator:
    """Encadena ``SessionBriefingEngine``, ``OptionsIntentRouter`` y ``PaperEntryPlanner``."""

    def __init__(
        self,
        briefing_engine: SessionBriefingEngine,
        intent_router: OptionsIntentRouter,
        entry_planner: PaperEntryPlanner,
        journal: OptionsPaperJournal | None = None,
    ) -> None:
        self._briefing = briefing_engine
        self._intent = intent_router
        self._planner = entry_planner
        self._journal = journal

    def _attach_visual_signal_to_briefing(
        self,
        briefing: dict[str, Any],
        *,
        caller_breach: dict[str, Any] | None,
        manual_entry_overrides: dict[str, Any] | None,
    ) -> None:
        """Enriquece briefing con ``visual_signal`` y fusiona ``breach_context`` antes de intent/entry."""
        mo = manual_entry_overrides if isinstance(manual_entry_overrides, dict) else {}
        tol = _optional_positive_float(mo.get("breach_tolerance"))
        breach_tol = float(tol) if tol is not None else 0.0

        def _summary(
            *,
            availability: str,
            adapter_out: dict[str, Any] | None = None,
            extra_notes: list[str] | None = None,
        ) -> dict[str, Any]:
            notes = list(extra_notes or [])
            det = (adapter_out or {}).get("details") if isinstance(adapter_out, dict) else {}
            det_notes = det.get("notes") if isinstance(det, dict) else []
            if isinstance(det_notes, list):
                notes.extend(str(x) for x in det_notes[:5])
            return {
                "availability": availability,
                "breach_detected": bool((adapter_out or {}).get("breach_detected")) if adapter_out else False,
                "critical_breach": bool((adapter_out or {}).get("critical_breach")) if adapter_out else False,
                "short_strike_breached": bool((adapter_out or {}).get("short_strike_breached")) if adapter_out else False,
                "breached_level_type": (det or {}).get("breached_level_type") if isinstance(det, dict) else None,
                "source": "visual_signal_adapter",
                "notes": notes[:8],
            }

        adapter_out: dict[str, Any] | None = None
        try:
            spot_raw = briefing.get("spot")
            spot_f = float(spot_raw) if spot_raw is not None else None
        except (TypeError, ValueError):
            spot_f = None

        if spot_f is None or spot_f <= 0:
            merged = dict(caller_breach or {})
            briefing["breach_context"] = merged
            briefing["visual_breach_flag"] = bool(merged.get("breach_detected"))
            briefing["visual_signal"] = _summary(availability="unavailable", extra_notes=["no_spot_in_briefing"])
            return

        is_0dte = str(briefing.get("dte_mode") or "").strip().lower() == "0dte"
        supports = briefing.get("support_levels") if isinstance(briefing.get("support_levels"), list) else []
        resists = briefing.get("resistance_levels") if isinstance(briefing.get("resistance_levels"), list) else []
        sup_f = [float(x) for x in supports if _optional_positive_float(x) is not None]
        res_f = [float(x) for x in resists if _optional_positive_float(x) is not None]

        try:
            adapter = VisualSignalAdapter(breach_tolerance=breach_tol)
            adapter_out = adapter.analyze_price_context(
                spot=spot_f,
                short_put_strike=_optional_positive_float(mo.get("short_put_strike")),
                short_call_strike=_optional_positive_float(mo.get("short_call_strike")),
                support_levels=sup_f or None,
                resistance_levels=res_f or None,
                is_0dte=is_0dte,
            )
            merged = _merge_breach_contexts(caller_breach, adapter_out)
            briefing["breach_context"] = merged
            briefing["visual_breach_flag"] = bool(merged.get("breach_detected"))
            briefing["visual_signal"] = _summary(availability="ok", adapter_out=adapter_out)
            if adapter_out.get("breach_detected"):
                qf = briefing.setdefault("quality_flags", [])
                if "visual_signal_adapter_breach" not in qf:
                    qf.append("visual_signal_adapter_breach")
        except Exception as exc:
            merged = dict(caller_breach or {})
            briefing["breach_context"] = merged
            briefing["visual_breach_flag"] = bool(merged.get("breach_detected"))
            briefing["visual_signal"] = _summary(
                availability="error",
                extra_notes=[f"adapter_exception:{exc!r}"],
            )

    def build_session_plan(
        self,
        *,
        symbol: str,
        direction: str | None = None,
        regime: str | None = None,
        gamma_regime: str | None = None,
        dte_mode: str | None = None,
        days_to_event: int | None = None,
        event_near: bool | None = None,
        spot: float | None = None,
        support_levels: list[float] | None = None,
        resistance_levels: list[float] | None = None,
        breach_context: dict[str, Any] | None = None,
        capital: float | None = None,
        manual_intent_overrides: dict[str, Any] | None = None,
        manual_entry_overrides: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        sym = str(symbol or "").strip().upper()
        pipeline_notes: list[str] = ["automation_paper_only_no_orders_no_live_broker"]
        pipeline_quality_flags: list[str] = []

        if not sym:
            pipeline_quality_flags.append("orchestrator:missing_symbol")

        dir_eff = (direction or "neutral").strip() or "neutral"
        reg_eff = (regime or "ranging").strip() or "ranging"
        event_eff = bool(event_near) if event_near is not None else False

        briefing: dict[str, Any] = {}
        if sym:
            briefing = self._briefing.build_briefing(
                sym,
                direction=dir_eff,
                regime=reg_eff,
                gamma_regime=gamma_regime,
                dte_mode=dte_mode,
                days_to_event=days_to_event,
                event_near=event_eff,
                spot=spot,
                support_levels=support_levels,
                resistance_levels=resistance_levels,
                breach_context=breach_context,
            )
            self._attach_visual_signal_to_briefing(
                briefing,
                caller_breach=breach_context,
                manual_entry_overrides=manual_entry_overrides,
            )
            qf = briefing.get("quality_flags") or []
            _options_metrics.record_pipeline_module(
                module="briefing",
                status=1.0 if not qf else 0.5,
            )
        else:
            pipeline_quality_flags.append("orchestrator:briefing_skipped_empty_symbol")
            _options_metrics.record_pipeline_module(module="briefing", status=0.0)

        intent: dict[str, Any] = {}
        if sym:
            intent = self._intent.build_intent(
                briefing,
                manual_overrides=manual_intent_overrides,
            )
            _options_metrics.record_pipeline_module(module="intent_router", status=1.0)
        else:
            intent = {
                "symbol": "",
                "allow_entry": False,
                "force_no_trade": True,
                "suppress_short_premium": True,
                "prefer_defined_risk": True,
                "entry_bias": "neutral",
                "market_bias": "neutral",
                "risk_posture": "blocked",
                "selector_inputs": {},
                "preferred_families": [],
                "blocked_families": list(KNOWN_STRATEGY_FAMILIES),
                "strategy_candidates": [],
                "recommended_strategy": None,
                "intent_notes": ["empty_symbol_pipeline_abort"],
                "intent_quality_flags": ["orchestrator:missing_symbol"],
            }
            _options_metrics.record_pipeline_module(module="intent_router", status=0.0)

        entry_plan = self._planner.build_entry_plan(
            intent,
            briefing=briefing if sym else None,
            capital=capital,
            manual_entry_overrides=manual_entry_overrides,
        )
        _options_metrics.record_pipeline_module(module="entry_planner", status=1.0 if entry_plan else 0.0)

        self._merge_child_flags(
            briefing,
            intent,
            entry_plan,
            pipeline_quality_flags,
            pipeline_notes,
        )

        entry_allowed = (
            bool(intent.get("allow_entry"))
            and not bool(intent.get("force_no_trade"))
            and str(entry_plan.get("entry") or "").lower() == "proposed"
        )

        flow_snap: dict[str, Any] | None = None
        if sym and _options_metrics.options_flow_bridge_enabled():
            spot_hint = float(spot) if spot is not None else 0.0
            if not spot_hint and isinstance(briefing, dict):
                spot_hint = float(briefing.get("spot") or 0.0)
            try:
                flow_snap = _options_metrics.try_fetch_options_flow_snapshot(sym, price_hint=spot_hint)
            except Exception as exc:
                flow_snap = {"available": False, "reason": f"orchestrator_options_flow_bridge:{exc}"}

        data: dict[str, Any] = {
            "automation_mode": "paper_only",
            "symbol": sym,
            "entry_allowed": entry_allowed,
            "briefing": briefing,
            "intent": intent,
            "entry_plan": entry_plan,
            "pipeline_notes": self._dedup(pipeline_notes),
            "pipeline_quality_flags": self._dedup(pipeline_quality_flags),
            "market_open_runtime": get_options_engine_market_open_runtime(),
        }
        if flow_snap is not None:
            data["options_flow_snapshot"] = flow_snap
        _attach_options_self_audit(data)
        attach_atlas_core_provenance_to_plan(data)
        if self._journal is not None:
            data["trace_id"] = self._journal.log_session_plan(
                data,
                notes=["paper_session_plan_orchestrator"],
            )
        jpath = self._journal.path if self._journal is not None else _options_metrics.default_journal_path()
        _options_metrics.record_session_plan(data, journal_path=jpath)
        _options_metrics.refresh_journal_from_disk(Path(jpath))
        _options_metrics.tick_pipeline_ages()
        return data

    def _merge_child_flags(
        self,
        briefing: dict[str, Any],
        intent: dict[str, Any],
        entry_plan: dict[str, Any],
        pipeline_quality_flags: list[str],
        pipeline_notes: list[str],
    ) -> None:
        for f in briefing.get("quality_flags") or []:
            pipeline_quality_flags.append(f"briefing:{f}")
        for n in briefing.get("operational_notes") or []:
            pipeline_notes.append(f"briefing_op:{n}")

        for f in intent.get("intent_quality_flags") or []:
            pipeline_quality_flags.append(f"intent:{f}")
        for n in intent.get("intent_notes") or []:
            pipeline_notes.append(f"intent:{n}")

        for f in entry_plan.get("entry_quality_flags") or []:
            pipeline_quality_flags.append(f"entry:{f}")
        for n in entry_plan.get("entry_notes") or []:
            pipeline_notes.append(f"entry:{n}")

    @staticmethod
    def _dedup(items: list[Any]) -> list[Any]:
        out: list[Any] = []
        seen: set[Any] = set()
        for item in items:
            if item in seen:
                continue
            seen.add(item)
            out.append(item)
        return out