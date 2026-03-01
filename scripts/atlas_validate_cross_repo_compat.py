from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Dict, List


BASE = Path(__file__).resolve().parent.parent
PAN = BASE / "_external" / "rauli-panaderia"
VIS = BASE / "_external" / "RAULI-VISION"
LOG = BASE / "logs" / "atlas_compatibility.log"
LOG.parent.mkdir(parents=True, exist_ok=True)


def _run(cmd: List[str], cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        capture_output=True,
        text=True,
        timeout=20,
    )


def _git_changed(repo: Path) -> List[str]:
    if not (repo / ".git").exists():
        return []
    a = _run(["git", "status", "--porcelain"], cwd=repo)
    out: List[str] = []
    for line in (a.stdout or "").splitlines():
        if len(line) < 4:
            continue
        out.append(line[3:].strip())
    return out


def _append_log(payload: Dict) -> None:
    with LOG.open("a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")


def main() -> int:
    result: Dict = {
        "pan_repo": str(PAN),
        "vision_repo": str(VIS),
        "ok": True,
        "checks": {},
        "reasons": [],
    }

    if not PAN.exists() or not VIS.exists():
        result["ok"] = False
        result["reasons"].append("missing_external_repos")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        _append_log(result)
        return 1

    pan_changed = _git_changed(PAN)
    pan_logic_changed = [
        p
        for p in pan_changed
        if p.startswith("backend/routes/")
        or p.startswith("backend/services/")
        or p == "backend/server.js"
        or p == "backend/database/init.js"
    ]
    result["checks"]["pan_changed_count"] = len(pan_changed)
    result["checks"]["pan_logic_changed_count"] = len(pan_logic_changed)
    result["checks"]["pan_logic_changed"] = pan_logic_changed[:50]

    inv_route = PAN / "backend" / "routes" / "inventory.js"
    if not inv_route.exists():
        result["ok"] = False
        result["reasons"].append("missing_pan_inventory_route")
    else:
        txt = inv_route.read_text(encoding="utf-8", errors="replace")
        has_adjustment = "router.post('/adjustment'" in txt
        result["checks"]["pan_inventory_adjustment_route"] = has_adjustment
        if not has_adjustment:
            result["ok"] = False
            result["reasons"].append("missing_inventory_adjustment_endpoint")

    vision_api_doc = VIS / "docs" / "API_RAULI-VISION.md"
    result["checks"]["vision_api_doc_exists"] = vision_api_doc.exists()
    if not vision_api_doc.exists():
        result["ok"] = False
        result["reasons"].append("missing_vision_api_doc")

    # Gate rule: if panaderia logic changed, compatibility checks must pass.
    if pan_logic_changed and not result["ok"]:
        result["checks"]["gate"] = "blocked"
    else:
        result["checks"]["gate"] = "pass"

    print(json.dumps(result, ensure_ascii=False, indent=2))
    _append_log(result)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
