import sqlite3,sys
p=sys.argv[1]
con=sqlite3.connect(p)
cur=con.cursor()
try:
    total=cur.execute("select count(*) from approvals").fetchone()[0]
    pending=cur.execute("select count(*) from approvals where status='pending'").fetchone()[0]
    print(f"total={total} pending={pending}")
    rows=cur.execute("select id,action,risk,status,created_ts from approvals order by created_ts desc limit 3").fetchall()
    for r in rows:
        print(r)
except Exception as e:
    print('ERR',e)
finally:
    con.close()
