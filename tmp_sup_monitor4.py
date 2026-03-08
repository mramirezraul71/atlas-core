import sqlite3, time

db = "C:/ATLAS_PUSH/data/autonomy_tasks.db"
con = sqlite3.connect(db)
cur = con.cursor()
start = cur.execute("select count(*) from autonomy_tasks where source='supervisor'").fetchone()[0]
print('start_total', start)
time.sleep(45)
end = cur.execute("select count(*) from autonomy_tasks where source='supervisor'").fetchone()[0]
print('end_total', end, 'delta', end-start)
last = cur.execute("select id,title,action_taken,created_at from autonomy_tasks where source='supervisor' order by created_at desc limit 3").fetchall()
for r in last: print(r)
