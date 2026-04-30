"""Bridge module to connect ClawdBOT AI with ATLAS central logs and gated actions."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict


SNAPSHOT_START = "=== SNAPSHOT_SAFE_START ==="
SNAPSHOT_END = "=== SNAPSHOT_SAFE_END ==="
DEFAULT_ALLOWED_REPOS = ("panaderia", "vision")
BLOCKED_PATTERNS = (
    "git reset --hard",
    "git clean -fd",
    "del /f /q",
    "rd /s /q",
    "shutdown ",
    "format ",
    "reboot",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _to_bool(value: str | None, default: bool) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def _truncate(text: str, limit: int = 5000) -> str:
    if len(text) <= limit:
        return text
    return text[:limit] + f"\n...[truncated {len(text) - limit} chars]"


def _is_placeholder(value: str) -> bool:
    v = (value or "").strip().upper()
    return (not v) or v.startswith("CHANGE_ME") or v in {"YOUR_TOKEN_HERE", "TU_TOKEN_PRIVADO"}


def _is_snapshot_marker_line(line: str, marker: str) -> bool:
    ln = (line or "").rstrip()
    if "CLAWD_BRIDGE" in ln:
        return False
    return ln.endswith(marker)


@dataclass
class ClawdBridgeConfig:
    repo_root: Path
    snapshot_script: Path
    snapshot_log: Path
    require_stable: bool = True
    require_token_for_evidence: bool = False
    allowed_repos: tuple[str, ...] = DEFAULT_ALLOWED_REPOS
    panaderia_repo: Path = Path("_external/rauli-panaderia")
    vision_repo: Path = Path("_external/RAULI-VISION")
    max_output_chars: int = 6000
    action_timeout_sec: int = 180

    @classmethod
    def from_env(cls) -> "ClawdBridgeConfig":
        repo_root = Path(__file__).resolve().parents[2]
        return cls(
            repo_root=repo_root,
            snapshot_script=Path(
                os.getenv("ATLAS_CLAWD_BRIDGE_SNAPSHOT_SCRIPT")
                or str(repo_root / "scripts" / "atlas_snapshot_safe.ps1")
            ).resolve(),
            snapshot_log=Path(
                os.getenv("ATLAS_CLAWD_BRIDGE_SNAPSHOT_LOG")
                or str(repo_root / "logs" / "snapshot_safe_diagnostic.log")
            ).resolve(),
            require_stable=_to_bool(os.getenv("ATLAS_CLAWD_BRIDGE_REQUIRE_STABLE"), True),
            require_token_for_evidence=_to_bool(
                os.getenv("ATLAS_CLAWD_BRIDGE_REQUIRE_TOKEN_FOR_EVIDENCE"), False
            ),
            allowed_repos=tuple(
                x.strip().lower()
                for x in (
                    os.getenv("ATLAS_CLAWD_BRIDGE_ALLOWED_REPOS")
                    or ",".join(DEFAULT_ALLOWED_REPOS)
                ).split(",")
                if x.strip()
            ),
            panaderia_repo=Path(
                os.getenv("ATLAS_CLAWD_BRIDGE_PANADERIA_REPO")
                or str(repo_root / "_external" / "rauli-panaderia")
            ).resolve(),
            vision_repo=Path(
                os.getenv("ATLAS_CLAWD_BRIDGE_VISION_REPO")
                or str(repo_root / "_external" / "RAULI-VISION")
            ).resolve(),
            max_output_chars=int(os.getenv("ATLAS_CLAWD_BRIDGE_MAX_OUTPUT_CHARS", "6000")),
            action_timeout_sec=int(os.getenv("ATLAS_CLAWD_BRIDGE_ACTION_TIMEOUT_SEC", "180")),
        )


class AtlasClawdBridge:
    """Central bridge for ClawdBOT evidence and gated code actions."""

    def __init__(self, config: ClawdBridgeConfig):
        self.config = config
        self.config.snapshot_log.parent.mkdir(parents=True, exist_ok=True)
        if not self.config.snapshot_log.exists():
            self.config.snapshot_log.write_text("", encoding="utf-8")

    def _write_log_line(self, event: str, payload: Dict[str, Any]) -> None:
        line = {
            "ts": _utc_now(),
            "event": event,
            "payload": payload,
        }
        with self.config.snapshot_log.open("a", encoding="utf-8") as f:
            f.write(f"{line['ts']} CLAWD_BRIDGE {json.dumps(line, ensure_ascii=False)}\n")

    def _expected_token(self) -> str:
        direct = (os.getenv("ATLAS_CENTRAL_CORE") or "").strip()
        if not _is_placeholder(direct):
            return direct
        fallback = (os.getenv("APPROVALS_CHAIN_SECRET") or "").strip()
        if not _is_placeholder(fallback):
            return fallback
        return ""

    def authorize(self, token: str | None, *, require_token: bool = True) -> Dict[str, Any]:
        expected = self._expected_token()
        provided = (token or os.getenv("ATLAS_CENTRAL_CORE") or "").strip()
        if not expected:
            return {
                "ok": not require_token,
                "reason": ("no_expected_token" if require_token else "token_not_configured_optional"),
            }
        if not provided:
            return {"ok": False, "reason": "missing_token"}
        if provided != expected:
            return {"ok": False, "reason": "invalid_token"}
        return {"ok": True, "reason": "authorized"}

    def _repo_map(self) -> Dict[str, Path]:
        return {
            "panaderia": self.config.panaderia_repo.resolve(),
            "vision": self.config.vision_repo.resolve(),
        }

    def repo_access(self, token: str | None) -> Dict[str, Any]:
        auth = self.authorize(token, require_token=True)
        repos = self._repo_map()
        result = {
            name: {"path": str(path), "exists": path.exists()}
            for name, path in repos.items()
        }
        return {
            "ok": bool(auth.get("ok")),
            "authorized": bool(auth.get("ok")),
            "auth_reason": auth.get("reason"),
            "repos": result,
            "allowed_repos": list(self.config.allowed_repos),
        }

    def run_snapshot_safe(self, timeout_sec: int = 240) -> Dict[str, Any]:
        script = self.config.snapshot_script
        if not script.exists():
            return {"ok": False, "error": f"snapshot_script_missing: {script}"}
        cmd = [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(script),
        ]
        try:
            proc = subprocess.run(
                cmd,
                cwd=str(self.config.repo_root),
                capture_output=True,
                text=True,
                timeout=timeout_sec,
            )
            return {
                "ok": proc.returncode == 0,
                "returncode": proc.returncode,
                "stdout": _truncate(proc.stdout or "", 1200),
                "stderr": _truncate(proc.stderr or "", 1200),
            }
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def parse_latest_snapshot(self) -> Dict[str, Any]:
        if not self.config.snapshot_log.exists():
            return {"ok": False, "stable": False, "error": "snapshot_log_missing"}

        lines = self.config.snapshot_log.read_text(
            encoding="utf-8",
            errors="replace",
        ).splitlines()
        if not lines:
            return {"ok": False, "stable": False, "error": "snapshot_log_empty"}

        start_indices = [
            i for i, ln in enumerate(lines) if _is_snapshot_marker_line(ln, SNAPSHOT_START)
        ]
        if not start_indices:
            return {"ok": False, "stable": False, "error": "snapshot_start_not_found"}
        start_idx = start_indices[-1]

        end_idx = None
        for i in range(start_idx, len(lines)):
            if _is_snapshot_marker_line(lines[i], SNAPSHOT_END):
                end_idx = i
        if end_idx is None:
            end_idx = len(lines) - 1

        block = lines[start_idx : end_idx + 1]

        def has_status(prefix: str) -> bool:
            needle = f"{prefix} => 200"
            return any(needle in ln for ln in block)

        push_ok = has_status("PUSH /health")
        nexus_ok = has_status("NEXUS /health")
        robot_ok = has_status("ROBOT /status")
        smoke_ok = any("SMOKE Results:" in ln and "0 failed" in ln for ln in block)
        end_marker = any(SNAPSHOT_END in ln for ln in block)

        stable = push_ok and nexus_ok and robot_ok and smoke_ok and end_marker
        return {
            "ok": True,
            "stable": stable,
            "checks": {
                "push_health_200": push_ok,
                "nexus_health_200": nexus_ok,
                "robot_status_200": robot_ok,
                "smoke_results_0_failed": smoke_ok,
                "end_marker_present": end_marker,
            },
            "window": {
                "start_line": start_idx + 1,
                "end_line": end_idx + 1,
                "lines": len(block),
            },
            "tail": [_truncate(x, 240) for x in block[-12:]],
        }

    def ensure_stable(self, *, run_snapshot: bool, raise_on_unstable: bool = True) -> Dict[str, Any]:
        snapshot_run = {"ok": True, "skipped": True}
        if run_snapshot:
            snapshot_run = self.run_snapshot_safe()

        status = self.parse_latest_snapshot()
        status_compact = {
            "ok": status.get("ok"),
            "stable": status.get("stable"),
            "checks": status.get("checks"),
            "window": status.get("window"),
        }
        payload = {
            "run_snapshot": run_snapshot,
            "snapshot_run": snapshot_run,
            "status": status_compact,
        }
        self._write_log_line("STABILITY_CHECK", payload)

        stable = bool(status.get("stable"))
        if self.config.require_stable and not stable and raise_on_unstable:
            raise RuntimeError("snapshot_safe_not_stable")
        return payload

    def write_evidence(
        self,
        *,
        source: str,
        evidence_type: str,
        message: str,
        payload: Dict[str, Any] | None,
        token: str | None,
    ) -> Dict[str, Any]:
        if self.config.require_token_for_evidence:
            auth = self.authorize(token, require_token=True)
            if not auth.get("ok"):
                raise PermissionError(str(auth.get("reason")))
        evidence = {
            "source": (source or "clawdbot-ai").strip()[:80],
            "type": (evidence_type or "runtime").strip()[:80],
            "message": (message or "").strip()[:1500],
            "payload": payload or {},
        }
        self._write_log_line("EVIDENCE", evidence)
        return {"ok": True, "evidence": evidence}

    def execute_action(
        self,
        *,
        action_id: str,
        target_repo: str,
        command: str,
        token: str | None,
        run_snapshot: bool = True,
        timeout_sec: int | None = None,
    ) -> Dict[str, Any]:
        auth = self.authorize(token, require_token=True)
        if not auth.get("ok"):
            return {"ok": False, "error": "unauthorized", "reason": auth.get("reason")}

        stability = self.ensure_stable(run_snapshot=run_snapshot, raise_on_unstable=False)
        if self.config.require_stable and not bool(
            (stability.get("status") or {}).get("stable")
        ):
            return {
                "ok": False,
                "error": "blocked_unstable_system",
                "stability": stability,
            }

        repo_key = (target_repo or "").strip().lower()
        if repo_key not in self.config.allowed_repos:
            return {"ok": False, "error": "repo_not_allowed", "target_repo": repo_key}

        repo_map = self._repo_map()
        repo_path = repo_map.get(repo_key)
        if not repo_path or not repo_path.exists():
            return {
                "ok": False,
                "error": "repo_not_found",
                "target_repo": repo_key,
                "path": str(repo_path) if repo_path else "",
            }

        cmd = (command or "").strip()
        if not cmd:
            return {"ok": False, "error": "empty_command"}
        cmd_l = cmd.lower()
        for pat in BLOCKED_PATTERNS:
            if pat in cmd_l:
                return {"ok": False, "error": "blocked_command_pattern", "pattern": pat}

        timeout = int(timeout_sec or self.config.action_timeout_sec)
        try:
            proc = subprocess.run(
                cmd,
                cwd=str(repo_path),
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
            result = {
                "ok": proc.returncode == 0,
                "action_id": (action_id or "").strip()[:120] or "clawd_action",
                "target_repo": repo_key,
                "cwd": str(repo_path),
                "returncode": proc.returncode,
                "stdout": _truncate(proc.stdout or "", self.config.max_output_chars),
                "stderr": _truncate(proc.stderr or "", self.config.max_output_chars),
                "stability": stability,
            }
            self._write_log_line("ACTION_EXECUTED", result)
            return result
        except Exception as exc:
            result = {
                "ok": False,
                "error": "action_exception",
                "detail": str(exc),
                "target_repo": repo_key,
            }
            self._write_log_line("ACTION_FAILED", result)
            return result

    def init_connection(self, token: str | None) -> Dict[str, Any]:
        stability = self.ensure_stable(run_snapshot=True, raise_on_unstable=False)
        repo_access = self.repo_access(token)
        message = (
            "atlas_clawd_bridge initialized"
            if (stability.get("status") or {}).get("stable")
            else "atlas_clawd_bridge initialized with unstable snapshot"
        )
        evidence = self.write_evidence(
            source="atlas_agent_init",
            evidence_type="bridge_init",
            message=message,
            payload={
                "stable": bool((stability.get("status") or {}).get("stable")),
                "repo_access": repo_access,
            },
            token=(token if self.config.require_token_for_evidence else None),
        )
        return {
            "ok": True,
            "stability": stability,
            "repo_access": repo_access,
            "evidence": evidence,
        }


_bridge_singleton: AtlasClawdBridge | None = None


def get_bridge() -> AtlasClawdBridge:
    global _bridge_singleton
    if _bridge_singleton is None:
        _bridge_singleton = AtlasClawdBridge(ClawdBridgeConfig.from_env())
    return _bridge_singleton


def _parse_json_input(value: str | None) -> Dict[str, Any]:
    txt = (value or "").strip()
    if not txt:
        return {}
    try:
        data = json.loads(txt)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _bool_arg(value: str | None, default: bool) -> bool:
    if value is None:
        return default
    return _to_bool(value, default)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="atlas_clawd_bridge CLI")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_status = sub.add_parser("status", help="Get bridge status and snapshot stability")
    p_status.add_argument("--run-snapshot", default="false")
    p_status.add_argument("--json", action="store_true")

    p_init = sub.add_parser("init", help="Initialize bridge connection now")
    p_init.add_argument("--token", default=None)
    p_init.add_argument("--json", action="store_true")

    p_evidence = sub.add_parser("evidence", help="Write evidence to central snapshot log")
    p_evidence.add_argument("--source", default="clawdbot-ai")
    p_evidence.add_argument("--type", dest="evidence_type", default="runtime")
    p_evidence.add_argument("--message", required=True)
    p_evidence.add_argument("--payload-json", default="{}")
    p_evidence.add_argument("--token", default=None)
    p_evidence.add_argument("--json", action="store_true")

    p_action = sub.add_parser("action", help="Execute gated action against panaderia/vision repo")
    p_action.add_argument("--action-id", required=True)
    p_action.add_argument("--target-repo", required=True, choices=["panaderia", "vision"])
    p_action.add_argument("--command", required=True)
    p_action.add_argument("--token", default=None)
    p_action.add_argument("--run-snapshot", default="true")
    p_action.add_argument("--timeout-sec", type=int, default=180)
    p_action.add_argument("--json", action="store_true")

    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    bridge = get_bridge()

    if args.cmd == "status":
        result = {
            "ok": True,
            "config": {
                "snapshot_script": str(bridge.config.snapshot_script),
                "snapshot_log": str(bridge.config.snapshot_log),
                "require_stable": bridge.config.require_stable,
                "allowed_repos": list(bridge.config.allowed_repos),
            },
            "stability": bridge.ensure_stable(
                run_snapshot=_bool_arg(getattr(args, "run_snapshot", "false"), False),
                raise_on_unstable=False,
            ),
        }
        print(json.dumps(result, ensure_ascii=False, indent=2 if args.json else None))
        return 0

    if args.cmd == "init":
        result = bridge.init_connection(token=args.token)
        print(json.dumps(result, ensure_ascii=False, indent=2 if args.json else None))
        return 0

    if args.cmd == "evidence":
        payload = _parse_json_input(args.payload_json)
        result = bridge.write_evidence(
            source=args.source,
            evidence_type=args.evidence_type,
            message=args.message,
            payload=payload,
            token=args.token,
        )
        print(json.dumps(result, ensure_ascii=False, indent=2 if args.json else None))
        return 0

    if args.cmd == "action":
        result = bridge.execute_action(
            action_id=args.action_id,
            target_repo=args.target_repo,
            command=args.command,
            token=args.token,
            run_snapshot=_bool_arg(args.run_snapshot, True),
            timeout_sec=args.timeout_sec,
        )
        print(json.dumps(result, ensure_ascii=False, indent=2 if args.json else None))
        return 0 if result.get("ok") else 1

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
