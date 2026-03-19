import sqlite3

con = sqlite3.connect("C:/ATLAS_PUSH/data/autonomy_tasks.db")
cur = con.cursor()

print("table_info autonomy_tasks")
for r in cur.execute("PRAGMA table_info(autonomy_tasks)"):
    print(r)

print("\nindexes autonomy_tasks")
for r in cur.execute("PRAGMA index_list(autonomy_tasks)"):
    print(r)

print("\ntriggers")
for r in cur.execute("SELECT name, sql FROM sqlite_master WHERE type='trigger'"):
    print(r[0], r[1])

con.close()
