"""Test mínimo de contrato para MemoryEngineV2.create_checkpoint.

Casos cubiertos:
1) summary + PRE            -> stage normalizado PRE_CAMBIO
2) reason + PRE             -> stage normalizado PRE_CAMBIO
3) summary + PRE_CAMBIO     -> stage preservado PRE_CAMBIO
4) summary + BASELINE       -> stage preservado BASELINE
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from memory_engine import MemoryEngineV2


def _run_case(
    engine: MemoryEngineV2,
    name: str,
    kwargs: dict,
    expected_stage: str,
) -> dict:
    cp = engine.create_checkpoint(**kwargs)
    checkpoint_id = cp.get("id")
    stage = cp.get("stage")
    summary = cp.get("summary")

    # Validación de persistencia del checkpoint
    exists_on_disk = False
    if isinstance(checkpoint_id, str) and checkpoint_id:
        stem = checkpoint_id.replace("ckpt_", "checkpoint_")
        candidates = list(engine.paths.checkpoints.glob(f"{stem}.json"))
        exists_on_disk = bool(candidates)

    ok = bool(checkpoint_id) and stage == expected_stage and exists_on_disk
    return {
        "name": name,
        "ok": ok,
        "checkpoint_id": checkpoint_id,
        "stage": stage,
        "expected_stage": expected_stage,
        "summary": summary,
        "exists_on_disk": exists_on_disk,
    }


def main() -> int:
    engine = MemoryEngineV2(root=REPO_ROOT / "memory_engine", project_root=REPO_ROOT)
    engine.bootstrap()

    cases = [
        (
            "case_1_summary_pre",
            {"stage": "PRE", "summary": "audit_contract_case_1"},
            "PRE_CAMBIO",
        ),
        (
            "case_2_reason_pre",
            {"stage": "PRE", "reason": "audit_contract_case_2"},
            "PRE_CAMBIO",
        ),
        (
            "case_3_summary_pre_cambio",
            {"stage": "PRE_CAMBIO", "summary": "audit_contract_case_3"},
            "PRE_CAMBIO",
        ),
        (
            "case_4_summary_baseline",
            {"stage": "BASELINE", "summary": "audit_contract_case_4"},
            "BASELINE",
        ),
    ]

    results: list[dict] = []
    for name, kwargs, expected_stage in cases:
        try:
            results.append(_run_case(engine, name, kwargs, expected_stage))
        except Exception as e:
            results.append(
                {
                    "name": name,
                    "ok": False,
                    "error": f"{type(e).__name__}: {e}",
                    "expected_stage": expected_stage,
                }
            )

    ok = all(r.get("ok") is True for r in results)

    print(json.dumps({"results": results}, ensure_ascii=False, indent=2))
    print(f"CONTRACT_TEST_OK={ok}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
