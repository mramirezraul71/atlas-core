import sqlite3, time

db = "C:/ATLAS_PUSH/data/autonomy_tasks.db"
con = sqlite3.connect(db)
cur = con.cursor()
start = cur.execute("select count(*) from autonomy_tasks where source='supervisor'").fetchone()[0]
print('start_total', start)
time.sleep(70)
end = cur.execute("select count(*) from autonomy_tasks where source='supervisor'").fetchone()[0]
recent = cur.execute("select id,title,created_at from autonomy_tasks where source='supervisor' order by created_at desc limit 5").fetchall()
print('end_total', end, 'delta', end-start)
for r in recent:
    print(r)
