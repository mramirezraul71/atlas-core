import sqlite3

db = "C:/ATLAS_PUSH/data/autonomy_tasks.db"
con = sqlite3.connect(db)
cur = con.cursor()
rows = cur.execute("select count(*) from autonomy_tasks where source='supervisor' and created_at >= datetime('now','-2 minutes')").fetchone()[0]
last = cur.execute("select id,title,created_at from autonomy_tasks where source='supervisor' order by created_at desc limit 5").fetchall()
print('created_last_2m', rows)
for r in last:
    print(r)
