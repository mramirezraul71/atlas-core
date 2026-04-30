from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.atlas_cloudflare_guard import (
    authorize_dashboard_request,
    is_cloudflare_edge_ip,
    refresh_cloudflare_ranges,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verifica IP contra rangos Cloudflare autorizados.")
    parser.add_argument("--ip", required=True, help="IP remota a validar")
    parser.add_argument("--refresh", action="store_true", help="Actualiza rangos Cloudflare antes de validar.")
    args = parser.parse_args()

    out = {"ip": args.ip}
    if args.refresh:
        ok, msg = refresh_cloudflare_ranges()
        out["refresh"] = {"ok": ok, "message": msg}

    out["is_cloudflare_edge_ip"] = is_cloudflare_edge_ip(args.ip)
    auth_ok, reason = authorize_dashboard_request(args.ip, {})
    out["authorize_dashboard_request"] = {"ok": auth_ok, "reason": reason}
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
