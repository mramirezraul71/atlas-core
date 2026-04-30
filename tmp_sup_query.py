import sqlite3

db = "C:/ATLAS_PUSH/data/autonomy_tasks.db"
con = sqlite3.connect(db)
con.row_factory = sqlite3.Row
cur = con.cursor()
rows = cur.execute("select status, priority, count(*) c from autonomy_tasks where source='supervisor' group by status,priority order by c desc").fetchall()
print('summary', [dict(r) for r in rows])
recent = cur.execute("select id,title,status,priority,created_at from autonomy_tasks where source='supervisor' order by created_at desc limit 20").fetchall()
print('recent')
for r in recent:
    print(dict(r))
