import sqlite3
p = r"C:\ATLAS_PUSH\_external\rauli-panaderia\backend\database\genesis.db"
con=sqlite3.connect(p)
cur=con.cursor()
cur.execute("PRAGMA table_info(employees)")
cols=[r[1] for r in cur.fetchall()]
print('COLUMNS', cols)
q=[]
for c in ['id','name','full_name','first_name','last_name','phone','email','role','position','active']:
    if c in cols: q.append(c)
if 'name' not in q and 'full_name' not in q and 'first_name' in cols:
    pass
sel=', '.join(q) if q else '*'
cur.execute(f"SELECT {sel} FROM employees")
rows=cur.fetchall()
print('ROWCOUNT', len(rows))
for r in rows:
    print(r)
con.close()
