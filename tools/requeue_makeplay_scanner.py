from __future__ import annotations

import sqlite3
from datetime import datetime, timezone


def main() -> None:
    db_path = "C:/ATLAS_PUSH/logs/atlas_sched.sqlite"
    now = datetime.now(timezone.utc).isoformat()
    conn = sqlite3.connect(db_path)
    try:
        cur = conn.cursor()
        cur.execute(
            "UPDATE jobs SET status='queued', next_run_ts=?, retries=0, last_error=NULL, updated_ts=? "
            "WHERE name='makeplay_scanner'",
            (now, now),
        )
        conn.commit()
        print("requeued", cur.rowcount)
    finally:
        conn.close()


if __name__ == "__main__":
    main()

