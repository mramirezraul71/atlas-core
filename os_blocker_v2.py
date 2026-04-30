"""
OS Complete Blocker v2
- Purga OS del journal cada 3s
- Bloquea que Tradier position OS genere phantoms
- Corre hasta que se reinicie ATLAS o se elimine el archivo stop flag
"""
import sqlite3
import json
import time
import os
import datetime

DB = r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"
STATE_PATH = r"C:\ATLAS_PUSH\atlas_code_quant\data\operation\operation_center_state.json"
STOP_FLAG = r"C:\ATLAS_PUSH\os_blocker_stop.flag"
MAX_RUNTIME = 10800  # 3 horas max
start = time.time()
purge_count = 0
cycle = 0

print(f"[{datetime.datetime.now()}] OS Complete Blocker v2 iniciado")
print(f"Para detener: crear archivo {STOP_FLAG}")

while time.time() - start < MAX_RUNTIME:
    cycle += 1
    if os.path.exists(STOP_FLAG):
        print(f"[{datetime.datetime.now()}] Stop flag detectado. Saliendo.")
        break
    
    try:
        # 1. Purgar OS del journal
        conn = sqlite3.connect(DB, timeout=10)
        cur = conn.cursor()
        cur.execute("""
            UPDATE trading_journal 
            SET status='closed', exit_time=datetime('now'), realized_pnl=0.0,
                post_mortem_text='PHANTOM_OS_RESTRICTED_TRADIER_CANT_CLOSE'
            WHERE status='open' AND symbol='OS' 
              AND (broker_order_ids_json IN ('{}','[]') OR broker_order_ids_json IS NULL)
        """)
        rows = cur.rowcount
        conn.commit()
        
        # 2. Contar opens totales
        cur.execute("SELECT COUNT(*) FROM trading_journal WHERE status='open'")
        total_open = cur.fetchone()[0]
        conn.close()
        
        if rows > 0:
            purge_count += rows
            print(f"[{datetime.datetime.now()}] Purged {rows} OS phantom(s). Total: {purge_count}. Open restantes: {total_open}")
        elif cycle % 20 == 0:
            print(f"[{datetime.datetime.now()}] Heartbeat: open={total_open}, purgados acumulados={purge_count}")
            
    except Exception as e:
        print(f"[{datetime.datetime.now()}] DB Error: {e}")
    
    time.sleep(3)

print(f"[{datetime.datetime.now()}] Blocker terminado. Total purgados: {purge_count}, ciclos: {cycle}")
