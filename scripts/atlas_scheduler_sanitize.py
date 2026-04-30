#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from atlas_fault_playbooks import execute_playbook, plan_playbook_for_event
from atlas_fault_snapshot import _run_scheduler_runtime_health


def main() -> int:
    events = _run_scheduler_runtime_health()
    targets = [
        event
        for event in events
        if str(event.get("fault_code") or "")
        in {"scheduler.job_legacy_broken", "scheduler.job_stale_failed", "scheduler.job_state_desync"}
    ]
    reports = []
    for event in targets:
        playbook = plan_playbook_for_event(event)
        if not playbook:
            continue
        reports.append(execute_playbook(playbook, emit=True, backoff_sec=0))

    print(
        json.dumps(
            {
                "legacy_broken_events": len(targets),
                "playbooks_executed": len(reports),
                "reports": reports,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
