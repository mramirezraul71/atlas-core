import sqlite3, os
p = r"C:\ATLAS_PUSH\_external\rauli-panaderia\backend\database\genesis.db"
print("exists", os.path.exists(p), "size", os.path.getsize(p) if os.path.exists(p) else -1)
con = sqlite3.connect(p)
cur = con.cursor()
cur.execute("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name")
for (name,) in cur.fetchall():
    print(name)
con.close()
