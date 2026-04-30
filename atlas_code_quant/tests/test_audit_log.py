from __future__ import annotations

import json
import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.audit_log import AuditLog


def test_audit_log_writes_jsonl(tmp_path: Path) -> None:
    p = tmp_path / "audit.jsonl"
    al = AuditLog(path=p)
    al.write("command_blocked", reason="x", command={"target": "quant", "action": "submit"})
    lines = p.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1
    row = json.loads(lines[0])
    assert row["event_type"] == "command_blocked"
    assert row["reason"] == "x"

