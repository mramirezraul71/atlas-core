import sqlite3
p=r'C:\ATLAS_PUSH\logs\atlas_approvals.sqlite'
con=sqlite3.connect(p)
cur=con.cursor()
for r in cur.execute("select id,status,resolved_ts,resolved_by from approvals where id='2ca7aec7-acd'").fetchall():
    print(r)
con.close()
