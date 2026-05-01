from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

CONTRACT_VERSION = 1
ALLOWED_AUTON_MODES = {"off", "paper_supervised", "paper_autonomous", "paper_aggressive"}
CORE_STATE_CONTRACT_KEYS = (
    "account_scope",
    "paper_only",
    "auton_mode",
    "executor_mode",
    "vision_mode",
    "operator_present",
    "screen_integrity_ok",
    "fail_safe_active",
    "fail_safe_reason",
    "autonomy_mode",
    "max_risk_per_trade_pct",
)


def repo_root_from(anchor: Path | None = None) -> Path:
    if anchor is None:
        anchor = Path(__file__).resolve()
    return anchor.resolve().parents[2]


def quant_operation_state_path(root: Path | None = None) -> Path:
    base = root or repo_root_from()
    return base / "atlas_code_quant" / "data" / "operation" / "operation_center_state.json"


def core_operation_state_path(root: Path | None = None) -> Path:
    base = root or repo_root_from()
    return base / "data" / "operation" / "operation_center_state.json"


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def write_json_atomic(path: Path, payload: dict[str, Any], *, ensure_ascii: bool = True) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=ensure_ascii, indent=2)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp_name, path)
    finally:
        try:
            if os.path.exists(tmp_name):
                os.unlink(tmp_name)
        except OSError:
            pass


def build_core_contract_payload(state: dict[str, Any], *, source_module: str) -> dict[str, Any]:
    payload = {key: state.get(key) for key in CORE_STATE_CONTRACT_KEYS}
    auton_mode = str(payload.get("auton_mode") or "off").strip().lower()
    payload["auton_mode"] = auton_mode if auton_mode in ALLOWED_AUTON_MODES else "off"
    payload["contract_version"] = CONTRACT_VERSION
    payload["source_module"] = source_module
    payload["updated_at"] = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    return payload


def publish_core_contract(
    state: dict[str, Any],
    *,
    core_state_path: Path | None = None,
    source_module: str,
) -> dict[str, Any]:
    target = core_state_path or core_operation_state_path()
    payload = build_core_contract_payload(state, source_module=source_module)
    write_json_atomic(target, payload, ensure_ascii=False)
    return payload


def update_quant_state(
    updates: dict[str, Any],
    *,
    quant_state_path: Path | None = None,
    core_state_path: Path | None = None,
    source_module: str = "atlas_operator_interface",
    ensure_ascii: bool = False,
) -> dict[str, Any]:
    target = quant_state_path or quant_operation_state_path()
    state = read_json(target)
    state.update(updates)
    write_json_atomic(target, state, ensure_ascii=ensure_ascii)
    core_state = publish_core_contract(
        state,
        core_state_path=core_state_path,
        source_module=source_module,
    )
    return {
        "path": str(target),
        "state": state,
        "core_path": str(core_state_path or core_operation_state_path()),
        "core_state": core_state,
    }


def combined_operation_state(
    *,
    quant_state_path: Path | None = None,
    core_state_path: Path | None = None,
) -> dict[str, Any]:
    q_path = quant_state_path or quant_operation_state_path()
    c_path = core_state_path or core_operation_state_path()
    return {
        "path": str(q_path),
        "state": read_json(q_path),
        "core_path": str(c_path),
        "core_state": read_json(c_path),
    }
