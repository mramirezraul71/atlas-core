import sqlite3
import os

db_path = 'logs/atlas_nervous.sqlite'
if os.path.exists(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    
    # Ver tablas
    cur.execute("SELECT name FROM sqlite_master WHERE type='table'")
    tables = [r[0] for r in cur.fetchall()]
    print('Tablas:', tables)
    
    # Ver incidentes abiertos
    if 'incidents' in tables:
        cur.execute('SELECT COUNT(*) FROM incidents WHERE status = "open"')
        open_count = cur.fetchone()[0]
        print('Incidentes abiertos en DB:', open_count)
        
        cur.execute('SELECT check_id, severity, message FROM incidents WHERE status = "open" LIMIT 10')
        for row in cur.fetchall():
            cid = row[0][:25] if row[0] else ''
            sev = row[1] if row[1] else ''
            msg = (row[2] or '')[:35]
            print(f'  [{sev:4}] {cid:25} {msg}')
    
    conn.close()
else:
    print('DB no encontrada')
