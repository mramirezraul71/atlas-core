import sqlite3
p = r"C:\ATLAS_PUSH\_external\rauli-panaderia\backend\database\genesis.db"
con=sqlite3.connect(p)
cur=con.cursor()
cur.execute("SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%' ORDER BY name")
tables=[r[0] for r in cur.fetchall()]
for t in tables:
    cur.execute(f"PRAGMA table_info({t})")
    cols=[r[1] for r in cur.fetchall()]
    m=[c for c in cols if any(k in c.lower() for k in ['phone','telefono','whatsapp','mobile','cel'])]
    if m:
        print(t+':'+','.join(m))
con.close()
