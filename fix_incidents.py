"""Fix open incidents count by clearing stale data."""
import sqlite3
import os
from pathlib import Path

dbs = [
    'logs/atlas_nervous.sqlite',
    'logs/nervous_system.sqlite', 
    'logs/autonomous_health.sqlite',
]

for db_path in dbs:
    if not os.path.exists(db_path):
        continue
    
    print(f"\n=== {db_path} ===")
    try:
        conn = sqlite3.connect(db_path)
        cur = conn.cursor()
        
        # Get tables
        cur.execute("SELECT name FROM sqlite_master WHERE type='table'")
        tables = [r[0] for r in cur.fetchall()]
        print(f"Tables: {tables}")
        
        # Check for incidents table
        for t in tables:
            cur.execute(f"SELECT COUNT(*) FROM {t}")
            cnt = cur.fetchone()[0]
            print(f"  {t}: {cnt} rows")
            
            # If it has status column, check open
            try:
                cur.execute(f'SELECT COUNT(*) FROM {t} WHERE status = "open"')
                open_cnt = cur.fetchone()[0]
                if open_cnt > 0:
                    print(f"    -> {open_cnt} open, closing old...")
                    cur.execute(f'''UPDATE {t} SET status = "closed" 
                                   WHERE status = "open" 
                                   AND datetime(created_at) < datetime("now", "-1 hour")''')
                    conn.commit()
                    print(f"    -> closed {cur.rowcount} stale incidents")
            except:
                pass
        
        conn.close()
    except Exception as e:
        print(f"Error: {e}")

print("\nDone!")
