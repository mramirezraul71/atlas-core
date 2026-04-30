"""Servidor canónico de métricas Prometheus para ATLAS-Quant."""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, r"C:\ATLAS_PUSH")

from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard


PORT = int(os.getenv("ATLAS_METRICS_PORT", "9090"))
SCOPE = os.getenv("ATLAS_METRICS_SCOPE", "paper").strip().lower() or "paper"
SYNC_SEC = max(1, int(os.getenv("ATLAS_METRICS_SYNC_SEC", "2")))


def main() -> None:
    dashboard = GrafanaDashboard()
    if not dashboard.start_metrics_server(PORT):
        raise SystemExit(1)
    print(f"[ATLAS-Metrics] Servidor canónico en http://localhost:{PORT}/metrics")
    print(f"[ATLAS-Metrics] Sincronizando snapshot Tradier cada {SYNC_SEC}s ({SCOPE})")
    while True:
        try:
            dashboard.sync_from_canonical(account_scope=SCOPE)
        except Exception as exc:
            print(f"[ATLAS-Metrics] sync error: {exc}")
        time.sleep(SYNC_SEC)


if __name__ == "__main__":
    main()
