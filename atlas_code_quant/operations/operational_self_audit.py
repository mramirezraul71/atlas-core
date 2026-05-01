"""Self-audit operativo ligero (paper-first): observación y logs, sin bloquear submit."""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Literal

logger = logging.getLogger("quant.operations.self_audit")

Severity = Literal["INFO", "WARN", "ERROR", "BLOCK"]
Category = Literal["risk", "integrity", "reconciliation", "config"]
Scope = Literal["paper", "live"]


@dataclass
class SelfAuditCheck:
    id: str
    category: Category
    severity: Severity
    message: str
    detail: dict[str, Any] | None = None


@dataclass
class SelfAuditResult:
    passed: bool
    overall_severity: Severity
    checks: list[SelfAuditCheck] = field(default_factory=list)
    ts_utc: str = ""
    scope: Scope = "paper"


@dataclass
class OperationalSelfAuditContext:
    settings: Any
    operation_config: dict[str, Any]
    market_open_snapshot: dict[str, Any]
    protocol: dict[str, Any]
    positions_summary: dict[str, Any] | None = None
    scope: Scope = "paper"


def operational_self_audit_enabled(settings: Any) -> bool:
    """QUANT_OPERATIONAL_SELF_AUDIT_ENABLED explícito; si no, solo paper por defecto."""
    raw = os.getenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED")
    if raw is not None and str(raw).strip() != "":
        return str(raw).strip().lower() not in {"0", "false", "no"}
    scope = str(getattr(settings, "tradier_default_scope", None) or "paper").strip().lower()
    return scope == "paper"


def _severity_rank(sev: Severity) -> int:
    return {"INFO": 0, "WARN": 1, "ERROR": 2, "BLOCK": 3}[sev]


def _rollup(checks: list[SelfAuditCheck]) -> tuple[Severity, bool]:
    if not checks:
        return "INFO", True
    max_sev: Severity = "INFO"
    for c in checks:
        if _severity_rank(c.severity) > _severity_rank(max_sev):
            max_sev = c.severity
    passed = max_sev not in ("ERROR", "BLOCK")
    return max_sev, passed


def run_operational_self_audit(ctx: OperationalSelfAuditContext) -> SelfAuditResult:
    ts = datetime.now(timezone.utc).isoformat()
    if not operational_self_audit_enabled(ctx.settings):
        return SelfAuditResult(passed=True, overall_severity="INFO", checks=[], ts_utc=ts, scope=ctx.scope)

    checks: list[SelfAuditCheck] = []

    mp = int(getattr(ctx.settings, "market_open_max_positions", 3) or 3)
    if not (1 <= mp <= 20):
        checks.append(
            SelfAuditCheck(
                id="risk.max_positions_range",
                category="risk",
                severity="ERROR",
                message="market_open_max_positions fuera de rango operativo 1-20",
                detail={"market_open_max_positions": mp},
            )
        )
    else:
        max_from_snap = ctx.market_open_snapshot.get("max_positions")
        if max_from_snap is not None and int(max_from_snap) != mp:
            checks.append(
                SelfAuditCheck(
                    id="risk.max_positions_mismatch",
                    category="risk",
                    severity="WARN",
                    message="Divergencia entre settings.market_open_max_positions y snapshot market_open",
                    detail={"settings": mp, "snapshot": max_from_snap},
                )
            )
        else:
            checks.append(
                SelfAuditCheck(
                    id="risk.max_positions_ok",
                    category="risk",
                    severity="INFO",
                    message="market_open_max_positions coherente",
                    detail={"market_open_max_positions": mp},
                )
            )

    if not bool(getattr(ctx.settings, "market_open_config_loaded", False)):
        checks.append(
            SelfAuditCheck(
                id="config.market_open_json",
                category="config",
                severity="WARN",
                message="market_open_config.json no cargado; usando defaults/env",
                detail=None,
            )
        )

    scope_lower = str(ctx.scope or "paper").strip().lower()
    if scope_lower == "paper" and bool(getattr(ctx.settings, "equity_kelly_live_enabled", False)):
        checks.append(
            SelfAuditCheck(
                id="risk.live_kelly_flag",
                category="risk",
                severity="WARN",
                message="equity_kelly_live_enabled=True con alcance paper en self-audit",
                detail=None,
            )
        )

    rec = ctx.operation_config.get("reconciliation")
    if isinstance(rec, dict) and str(rec.get("state") or "").lower() == "failed":
        checks.append(
            SelfAuditCheck(
                id="reconciliation.state",
                category="reconciliation",
                severity="WARN",
                message="reconciliation.state=failed en snapshot de configuración",
                detail=dict(rec),
            )
        )

    if not isinstance(ctx.protocol, dict) or not ctx.protocol:
        checks.append(
            SelfAuditCheck(
                id="integrity.protocol_payload",
                category="integrity",
                severity="WARN",
                message="Protocolo de self-audit vacío o no dict; usando fallback interno",
                detail=None,
            )
        )

    overall, passed = _rollup(checks)
    if overall == "ERROR":
        logger.error(
            "operational_self_audit scope=%s overall=%s checks=%d",
            ctx.scope,
            overall,
            len(checks),
        )
    elif overall == "WARN":
        logger.warning(
            "operational_self_audit scope=%s overall=%s checks=%d",
            ctx.scope,
            overall,
            len(checks),
        )
    return SelfAuditResult(passed=passed, overall_severity=overall, checks=checks, ts_utc=ts, scope=ctx.scope)
