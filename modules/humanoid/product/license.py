"""Optional license validation (HMAC). Does not block when PRODUCT_MODE=false or PRODUCT_LICENSE_REQUIRED=false."""
from __future__ import annotations

import hmac
import os
from pathlib import Path
from typing import Any, Dict


def _repo_root() -> Path:
    p = os.getenv("ATLAS_REPO_PATH") or os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH"
    return Path(p).resolve()


def _license_required() -> bool:
    return os.getenv("PRODUCT_LICENSE_REQUIRED", "false").strip().lower() in ("1", "true", "yes")


def _product_mode() -> bool:
    return os.getenv("PRODUCT_MODE", "false").strip().lower() in ("1", "true", "yes")


def _secret() -> str:
    return os.getenv("PRODUCT_LICENSE_SECRET", "atlas-local-secret-change-in-production").strip()


def validate_license() -> Dict[str, Any]:
    """
    If PRODUCT_LICENSE_REQUIRED=false or PRODUCT_MODE=false: valid=True.
    Else: read license.key, verify HMAC(edition|expiry, secret). Returns {valid, message, edition}.
    """
    if not _license_required():
        return {"valid": True, "message": "license not required", "edition": os.getenv("PRODUCT_EDITION", "community")}
    if not _product_mode():
        return {"valid": True, "message": "dev mode", "edition": "community"}
    key_path = _repo_root() / "license.key"
    if not key_path.exists():
        return {"valid": False, "message": "license.key not found", "edition": None}
    try:
        raw = key_path.read_text(encoding="utf-8").strip()
        parts = raw.split("|")
        if len(parts) < 3:
            return {"valid": False, "message": "invalid format", "edition": None}
        edition, expiry, sig = parts[0].strip(), parts[1].strip(), parts[2].strip()
        payload = f"{edition}|{expiry}"
        expected = hmac.new(_secret().encode(), payload.encode(), "sha256").hexdigest()
        if not hmac.compare_digest(sig, expected):
            return {"valid": False, "message": "invalid signature", "edition": None}
        # Optional: check expiry (YYYY-MM-DD)
        if expiry and expiry != "never":
            from datetime import datetime, timezone
            try:
                exp_dt = datetime.strptime(expiry, "%Y-%m-%d").replace(tzinfo=timezone.utc)
                if datetime.now(timezone.utc) > exp_dt:
                    return {"valid": False, "message": "license expired", "edition": edition}
            except ValueError:
                pass
        return {"valid": True, "message": "ok", "edition": edition}
    except Exception as e:
        return {"valid": False, "message": str(e), "edition": None}


def license_status() -> Dict[str, Any]:
    """Convenience: validate_license() with status key for UI."""
    r = validate_license()
    return {"status": "valid" if r.get("valid") else "invalid", **r}
