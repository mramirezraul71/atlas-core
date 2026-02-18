from __future__ import annotations

import sqlite3
from datetime import datetime, timezone


def main() -> None:
    db_path = "C:/ATLAS_PUSH/logs/atlas_sched.sqlite"
    disable_names = {
        # Estos jobs pueden modificar el repo (commit/push/rebase). Desactivar para auditoría/estabilidad.
        "repo_monitor_after_fix",
        "nightly_repo_scan",
    }
    now = datetime.now(timezone.utc).isoformat()

    conn = sqlite3.connect(db_path)
    try:
        cur = conn.cursor()
        cur.execute("SELECT id, name, enabled, status FROM jobs")
        rows = cur.fetchall()
        hit = [r for r in rows if (r[1] or "") in disable_names]
        for jid, name, enabled, status in hit:
            cur.execute(
                "UPDATE jobs SET enabled = 0, status = 'paused', updated_ts = ? WHERE id = ?",
                (now, jid),
            )
        conn.commit()
        print("disabled", len(hit), "jobs:", [r[1] for r in hit])
    finally:
        conn.close()


if __name__ == "__main__":
    main()

