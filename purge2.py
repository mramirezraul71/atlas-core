
import sqlite3, time, os

db_path = r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"
print(f"DB path: {db_path}")
print(f"DB exists: {os.path.exists(db_path)}")

success = False
for attempt in range(10):
    try:
        conn = sqlite3.connect(db_path, timeout=20)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA busy_timeout=15000")
        conn.isolation_level = None  # autocommit para evitar locks
        
        cur = conn.cursor()
        cur.execute("SELECT COUNT(*) FROM trading_journal")
        before = cur.fetchone()[0]
        print(f"Before: {before}")
        
        cur.execute("DELETE FROM trading_journal WHERE DATE(entry_time) = DATE('now','localtime') AND status = 'open' AND realized_pnl = 0.0")
        deleted = cur.rowcount
        
        cur.execute("SELECT COUNT(*) FROM trading_journal")
        after = cur.fetchone()[0]
        
        cur.execute("SELECT COUNT(*) FROM trading_journal WHERE entry_price < 0")
        neg = cur.fetchone()[0]
        
        conn.close()
        print(f"PURGE OK | antes={before} | eliminados={deleted} | despues={after} | neg_prices={neg}")
        success = True
        break
    except Exception as e:
        print(f"attempt {attempt+1} error: {type(e).__name__}: {e}")
        time.sleep(4)

if not success:
    print("PURGE FAILED after 10 attempts")
