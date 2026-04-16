"""
Monitor Primer Trade Real ATLAS
- Detecta cuando aparece el primer trade con broker_order_ids real
- Notifica por Telegram
- Se autodestruye después de detectar o de 6 horas
"""
import sqlite3
import time
import datetime
import requests
import json

DB = r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"
TELEGRAM_TOKEN = "7956423194:AAG5K_idhDp-vtuBhMC46toFjV9ejBRr_4s"
CHAT_ID = "1749113793"
STOP_FLAG = r"C:\ATLAS_PUSH\trade_monitor_stop.flag"
MAX_RUNTIME = 21600  # 6 horas
start = time.time()
known_real_trades = set()

def send_telegram(msg):
    try:
        r = requests.post(
            f"https://api.telegram.org/bot{TELEGRAM_TOKEN}/sendMessage",
            data={"chat_id": CHAT_ID, "text": msg},
            timeout=10
        )
        return r.json().get("ok", False)
    except Exception as e:
        print(f"Telegram error: {e}")
        return False

def get_real_trades():
    try:
        conn = sqlite3.connect(DB, timeout=10)
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute("""
            SELECT id, symbol, strategy_type, status, broker_order_ids_json, 
                   entry_price, realized_pnl, entry_time
            FROM trading_journal 
            WHERE DATE(entry_time) = DATE('now')
              AND broker_order_ids_json NOT IN ('{}', '[]')
              AND broker_order_ids_json IS NOT NULL
            ORDER BY entry_time DESC
        """)
        rows = [dict(r) for r in cur.fetchall()]
        conn.close()
        return rows
    except Exception as e:
        print(f"DB error: {e}")
        return []

print(f"[{datetime.datetime.now()}] Monitor iniciado - esperando primer trade real")

import os
if os.path.exists(STOP_FLAG):
    os.remove(STOP_FLAG)

cycle = 0
while time.time() - start < MAX_RUNTIME:
    cycle += 1
    
    if os.path.exists(STOP_FLAG):
        print(f"[{datetime.datetime.now()}] Stop flag. Saliendo.")
        break
    
    trades = get_real_trades()
    new_trades = [t for t in trades if t['id'] not in known_real_trades]
    
    for trade in new_trades:
        known_real_trades.add(trade['id'])
        now_edt = datetime.datetime.utcnow() - datetime.timedelta(hours=4)
        msg = f"""ATLAS PRIMER TRADE REAL POST-LIMPIEZA

Simbolo: {trade['symbol']}
Estrategia: {trade['strategy_type']}
Entry: ${trade['entry_price']}
Status: {trade['status']}
Broker IDs: {trade['broker_order_ids_json'][:50]}
Hora: {now_edt.strftime('%H:%M:%S')} EDT

Sistema funcionando correctamente. Primer trade Fase 0 confirmado."""
        
        ok = send_telegram(msg)
        print(f"[{datetime.datetime.now()}] TRADE DETECTADO: {trade['symbol']} | Telegram: {'OK' if ok else 'FAIL'}")
        print(f"  broker_ids: {trade['broker_order_ids_json']}")
    
    if cycle % 30 == 0:  # Heartbeat cada 2.5 min
        elapsed = int(time.time() - start)
        print(f"[{datetime.datetime.now()}] Heartbeat: cycle={cycle}, real_trades={len(known_real_trades)}, elapsed={elapsed}s")
    
    time.sleep(5)

print(f"[{datetime.datetime.now()}] Monitor terminado. Total trades detectados: {len(known_real_trades)}")
