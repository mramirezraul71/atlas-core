"""Compare before/after and confirm 'done' (hash or simple diff)."""
from __future__ import annotations

import hashlib
from typing import Any, Dict, Optional, Tuple

from .capture import capture_screen


def capture_hash(region: Optional[Tuple[int, int, int, int]] = None) -> Tuple[Optional[str], str]:
    """Capture and return hex hash of image. Returns (hash, error)."""
    png, err = capture_screen(region=region)
    if err or not png:
        return None, err or "no data"
    h = hashlib.sha256(png).hexdigest()
    return h, ""


def verify_done(before_hash: str, after_hash: str, expect_change: bool = True) -> Dict[str, Any]:
    """
    Compare before/after hashes. expect_change=True -> done if hashes differ.
    Returns {done, same, message}.
    """
    same = before_hash == after_hash
    done = (not same) if expect_change else same
    return {
        "done": done,
        "same": same,
        "message": "change detected" if (expect_change and not same) else ("unchanged" if same else "no change"),
    }


def verify_with_capture(before_hash: Optional[str] = None, region: Optional[Tuple[int, int, int, int]] = None) -> Dict[str, Any]:
    """Capture now, compare to before_hash. If before_hash None, just return current hash."""
    h, err = capture_hash(region=region)
    if err:
        return {"ok": False, "hash": None, "done": False, "error": err}
    if before_hash is None:
        return {"ok": True, "hash": h, "done": False, "error": None}
    v = verify_done(before_hash, h or "", expect_change=True)
    return {"ok": True, "hash": h, "done": v["done"], "same": v["same"], "error": None}
