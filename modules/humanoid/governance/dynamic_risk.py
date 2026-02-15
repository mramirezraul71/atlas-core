from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class RiskAssessment:
    risk: str  # low|medium|high|critical
    reason: str
    signature: str
    requires_approval: bool


_RISK_ORDER = {"low": 0, "medium": 1, "med": 1, "high": 2, "critical": 3}


def _sig(text: str) -> str:
    t = (text or "").strip().lower()[:400]
    return hashlib.sha256(t.encode("utf-8", errors="ignore")).hexdigest()[:16]


def _max_risk(a: str, b: str) -> str:
    ra = _RISK_ORDER.get((a or "low").lower(), 0)
    rb = _RISK_ORDER.get((b or "low").lower(), 0)
    inv = {0: "low", 1: "medium", 2: "high", 3: "critical"}
    return inv[max(ra, rb)]


_RE_DESTRUCTIVE = re.compile(
    r"\b("
    r"rm\s+-rf|mkfs|format(\s+[a-z]:)?|diskpart|bcdedit|shutdown|reboot|restart-computer|"
    r"del\s+/f\s+/s|rd\s+/s\s+/q|remove-item\s+.*-recurse\s+.*-force|"
    r"drop\s+database|truncate\s+table|delete\s+from"
    r")\b",
    re.IGNORECASE,
)
_RE_NETWORK = re.compile(r"\b(netsh|route\s+add|iptables|ufw|firewall|portproxy|ngrok)\b", re.IGNORECASE)
_RE_SYSTEM_PATH = re.compile(r"\b([a-z]:\\windows\\|[a-z]:\\windows\\system32\\)\b", re.IGNORECASE)
_RE_SECRETS = re.compile(r"\b(credenciales\.txt|\.env|id_rsa|token|apikey|secret)\b", re.IGNORECASE)
_RE_GIT_DANGEROUS = re.compile(r"\bgit\s+(reset\s+--hard|clean\s+-fd|push\s+--force|rebase)\b", re.IGNORECASE)


def assess_shell_command(command: str, *, cwd: str = "") -> RiskAssessment:
    cmd = (command or "").strip()
    if not cmd:
        return RiskAssessment(risk="low", reason="empty_command", signature=_sig("empty"), requires_approval=False)

    risk = "low"
    reason = "ok"
    if _RE_DESTRUCTIVE.search(cmd):
        risk, reason = "critical", "destructive_command"
    if _RE_GIT_DANGEROUS.search(cmd):
        risk, reason = _max_risk(risk, "high"), "dangerous_git_operation"
    if _RE_NETWORK.search(cmd):
        risk, reason = _max_risk(risk, "high"), "network_or_firewall_change"
    if _RE_SYSTEM_PATH.search(cmd) or _RE_SYSTEM_PATH.search(cwd or ""):
        risk, reason = _max_risk(risk, "high"), "system_path_target"
    if _RE_SECRETS.search(cmd):
        risk, reason = _max_risk(risk, "high"), "secrets_sensitive_target"

    requires = _RISK_ORDER.get(risk, 0) >= _RISK_ORDER["high"]
    return RiskAssessment(risk=risk, reason=reason, signature=_sig(f"shell:{risk}:{reason}:{cmd}"), requires_approval=requires)


def assess_action(action_kind: str, context: Optional[Dict[str, Any]] = None) -> RiskAssessment:
    k = (action_kind or "").strip().lower()
    ctx = context or {}
    if k in ("shell_exec", "shell"):
        return assess_shell_command(str(ctx.get("command") or ""), cwd=str(ctx.get("cwd") or ""))
    # Default conservative: unknown actions are medium (no approval), unless explicitly flagged
    base = str(ctx.get("risk") or "").strip().lower()
    if base in ("high", "critical"):
        return RiskAssessment(risk=base, reason="explicit_risk", signature=_sig(f"{k}:{base}"), requires_approval=True)
    return RiskAssessment(risk="medium", reason="default_medium", signature=_sig(f"{k}:default"), requires_approval=False)

