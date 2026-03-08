import sqlite3

db = "C:/ATLAS_PUSH/data/autonomy_tasks.db"
con = sqlite3.connect(db)
cur = con.cursor()

rows = cur.execute(
    """
    select id, title, action_taken, length(coalesce(action_taken, '')) as action_len, created_at
    from autonomy_tasks
    where source='supervisor'
    order by created_at desc
    limit 20
    """
).fetchall()

print("last_20_supervisor_action_taken")
for r in rows:
    print(r)

non_empty = cur.execute(
    """
    select count(*)
    from autonomy_tasks
    where source='supervisor' and length(coalesce(action_taken,'')) > 0
    """
).fetchone()[0]

print("non_empty_action_taken_total", non_empty)
con.close()
