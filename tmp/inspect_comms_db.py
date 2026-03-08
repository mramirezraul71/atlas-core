import sqlite3
from pathlib import Path

p = Path(r"c:\ATLAS_PUSH\logs\atlas_comms_hub.sqlite")
con = sqlite3.connect(p)
cur = con.cursor()
cur.execute("SELECT name FROM sqlite_master WHERE type='table'")
tables = [r[0] for r in cur.fetchall()]
print('TABLES:', tables)

for t in tables:
    try:
        cur.execute(f'PRAGMA table_info({t})')
        cols = [r[1] for r in cur.fetchall()]
        print(f'\n[{t}] columns={cols}')
        cur.execute(f'SELECT * FROM {t} ORDER BY rowid DESC LIMIT 5')
        rows = cur.fetchall()
        for row in rows:
            print(' ', row)
    except Exception as e:
        print('ERR', t, e)

hits = []
for t in tables:
    cur.execute(f'PRAGMA table_info({t})')
    cols = [r[1] for r in cur.fetchall()]
    for c in cols:
        try:
            cur.execute(
                f"SELECT rowid, CAST({c} AS TEXT) FROM {t} WHERE lower(CAST({c} AS TEXT)) LIKE ? OR CAST({c} AS TEXT) LIKE ? LIMIT 50",
                ('%bety%', '%+53%'),
            )
            for rid, val in cur.fetchall():
                hits.append((t, c, rid, val))
        except Exception:
            pass

print('\nHITS:', len(hits))
for h in hits:
    print(h)

con.close()
