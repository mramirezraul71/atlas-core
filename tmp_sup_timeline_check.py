import sqlite3

con = sqlite3.connect("C:/ATLAS_PUSH/data/autonomy_tasks.db")
cur = con.cursor()

rows = cur.execute(
    """
    select event, kind, result, ts
    from autonomy_timeline
    where event like 'supervisor_%'
    order by id desc
    limit 30
    """
).fetchall()

print("last_supervisor_timeline_events")
for r in rows:
    print(r)

recent_tg = cur.execute(
    """
    select count(*)
    from autonomy_timeline
    where event like 'supervisor_telegram:%'
      and ts >= datetime('now','-30 minutes')
    """
).fetchone()[0]
print("recent_supervisor_telegram_30m", recent_tg)

con.close()
