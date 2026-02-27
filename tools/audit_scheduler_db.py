from __future__ import annotations

import sqlite3


def main() -> None:
    db = "C:/ATLAS_PUSH/logs/atlas_sched.sqlite"
    conn = sqlite3.connect(db)
    try:
        cur = conn.cursor()
        cur.execute(
            "SELECT name, kind, enabled, status, interval_seconds, last_run_ts, next_run_ts, COALESCE(last_error, '') "
            "FROM jobs ORDER BY name"
        )
        rows = cur.fetchall()
        print("jobs", len(rows))
        for name, kind, enabled, status, interval_seconds, last_run_ts, next_run_ts, last_error in rows:
            name = name or ""
            kind = kind or ""
            last_run_ts = last_run_ts or ""
            next_run_ts = next_run_ts or ""
            last_error = last_error or ""

            if any(
                k in name
                for k in (
                    "makeplay",
                    "ans",
                    "nervous",
                    "repo",
                    "approvals",
                    "workshop",
                    "autonomy",
                    "update",
                    "world_state",
                )
            ):
                print(
                    "{:<24} kind={:<18} en={} st={:<8} iv={:<4} last={:<19} next={:<19} err={}".format(
                        name,
                        kind,
                        enabled,
                        status,
                        interval_seconds,
                        last_run_ts[:19],
                        next_run_ts[:19],
                        last_error[:80],
                    )
                )
    finally:
        conn.close()


if __name__ == "__main__":
    main()

