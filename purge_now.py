import sqlite3, sys

db = r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"

try:
    conn = sqlite3.connect(db, timeout=20)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA busy_timeout=15000")
    conn.isolation_level = None  # autocommit
    cur = conn.cursor()
    cur.execute("SELECT COUNT(*) FROM trading_journal")
    before = cur.fetchone()[0]
    cur.execute(
        "DELETE FROM trading_journal "
        "WHERE DATE(entry_time) = DATE('now','localtime') "
        "AND status = 'open' "
        "AND realized_pnl = 0.0"
    )
    deleted = cur.rowcount
    cur.execute("SELECT COUNT(*) FROM trading_journal")
    after = cur.fetchone()[0]
    conn.close()
    print(f"Antes: {before} | Eliminados: {deleted} | Despues: {after}")
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
