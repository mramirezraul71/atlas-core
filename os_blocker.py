"""
OS Phantom Blocker - Purga OS del journal en tiempo real (loop 3s)
Ejecutar en background mientras Tradier sandbox no permite cerrar OS
"""
import sqlite3
import time
import datetime

DB = r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"
MAX_RUNTIME = 7200  # 2 horas max
start = time.time()
purge_count = 0

print(f"[{datetime.datetime.now()}] OS Phantom Blocker iniciado")

while time.time() - start < MAX_RUNTIME:
    try:
        conn = sqlite3.connect(DB, timeout=10)
        cur = conn.cursor()
        cur.execute("""
            UPDATE trading_journal 
            SET status='closed', exit_time=datetime('now'), realized_pnl=0.0,
                post_mortem_text='PHANTOM_OS_BLOCKED_TRADIER_RESTRICTED'
            WHERE status='open' AND symbol='OS' 
              AND (broker_order_ids_json IN ('{}','[]') OR broker_order_ids_json IS NULL)
        """)
        rows = cur.rowcount
        conn.commit()
        conn.close()
        if rows > 0:
            purge_count += rows
            print(f"[{datetime.datetime.now()}] Purged {rows} OS phantom(s). Total: {purge_count}")
    except Exception as e:
        print(f"[{datetime.datetime.now()}] Error: {e}")
    time.sleep(3)

print(f"[{datetime.datetime.now()}] Blocker terminado. Total purgados: {purge_count}")
