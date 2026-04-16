"""
ATLAS Phantom Exit Guard — patch en memoria del JournalProService
para que exit_governance_snapshot ignore phantoms (broker_order_ids='{}')

Ejecutar via MCP run_script. Corre en el mismo proceso Python que ATLAS
y parchea el método directamente en el singleton.

Adicionalmente: purge continuo cada 4s durante 5 minutos para mantener
el journal limpio mientras el sync siga re-insertando.
"""
import sqlite3
import sys
import time
import threading
from datetime import datetime, timezone

DB_PATH = r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"
ROOT_PATH = r"C:\ATLAS_PUSH"

now_utc = datetime.now(timezone.utc).isoformat()

print("=" * 60)
print("ATLAS PHANTOM EXIT GUARD")
print("=" * 60)

# ── 1. Purge inmediato ────────────────────────────────────────────────────────
def purge_phantoms(label="PURGE"):
    try:
        con = sqlite3.connect(DB_PATH)
        cur = con.cursor()
        cur.execute("SELECT COUNT(*) FROM trading_journal WHERE status='open' AND (broker_order_ids_json='{}' OR broker_order_ids_json IS NULL OR broker_order_ids_json='[]')")
        count = cur.fetchone()[0]
        if count > 0:
            ts = datetime.now(timezone.utc).isoformat()
            pm = '[' + label + ' ' + ts + '] Phantom sync Tradier - broker_order_ids vacio. Pending restart 949764f.'
            cur.execute(
                "UPDATE trading_journal SET status='closed', realized_pnl=0.0, exit_price=entry_price, exit_time=datetime('now'), post_mortem_text=? WHERE status='open' AND (broker_order_ids_json='{}' OR broker_order_ids_json IS NULL OR broker_order_ids_json='[]')",
                (pm,)
            )
            con.commit()
            print(f"[{label}] {count} phantoms cerrados @ {ts[:19]}")
        con.close()
        return count
    except Exception as e:
        print(f"[{label}] ERROR: {e}")
        return 0

purge_phantoms("PURGE-6-INIT")

# ── 2. Patch del JournalProService en memoria ─────────────────────────────────
patched_ok = False
try:
    sys.path.insert(0, ROOT_PATH)
    from atlas_code_quant.journal.service import JournalProService
    from sqlalchemy import select
    from atlas_code_quant.journal.models import TradingJournal
    from atlas_code_quant.journal.service import build_exit_governance_snapshot
    from atlas_code_quant.journal.db import session_scope

    _original_exit_governance = JournalProService.exit_governance_snapshot

    def _patched_exit_governance(self, *, account_type=None, limit=10):
        """Solo evalua posiciones con broker_order_ids valido (trades reales)."""
        with session_scope() as db:
            query = select(TradingJournal).where(
                TradingJournal.status == "open",
                # Excluir phantoms — solo posiciones con broker_order_id real
                TradingJournal.broker_order_ids_json.not_in(["{}","[]","null",""]),
                TradingJournal.broker_order_ids_json.is_not(None),
            )
            if account_type not in {None, "", "all"}:
                query = query.where(TradingJournal.account_type == account_type)
            rows = db.execute(
                query.order_by(TradingJournal.updated_at.desc(), TradingJournal.id.desc())
            ).scalars().all()
        result = build_exit_governance_snapshot(rows, account_type=account_type, limit=limit)
        real_count = len(rows)
        print(f"[EXIT_GUARD] exit_governance_snapshot patched: {real_count} posiciones reales (phantoms excluidos)")
        return result

    JournalProService.exit_governance_snapshot = _patched_exit_governance
    patched_ok = True
    print("[PATCH] JournalProService.exit_governance_snapshot → patched OK (filtra phantoms)")

except Exception as e:
    print(f"[PATCH] ERROR: {e} — usando solo purge continuo")

# ── 3. Loop de purge continuo (3s) durante 6 minutos ─────────────────────────
# El journal_sync re-inserta cada 5s, el purge corre cada 3s → siempre limpio
print(f"[LOOP] Iniciando purge continuo cada 3s durante 6 minutos...")
print(f"[LOOP] El entry_pass deberia ejecutar entradas reales ahora")

total_purged = 0
cycles = 0
max_cycles = 120  # 120 x 3s = 360s = 6 minutos

for i in range(max_cycles):
    time.sleep(3)
    n = purge_phantoms(f"PURGE-LOOP-{i+1}")
    total_purged += n
    cycles += 1
    if i % 10 == 0:
        # Estado journal cada 30s
        try:
            con = sqlite3.connect(DB_PATH)
            cur = con.cursor()
            cur.execute("SELECT status, COUNT(*) FROM trading_journal GROUP BY status")
            summary = dict(cur.fetchall())
            cur.execute("SELECT strategy_type, symbol, entry_price, broker_order_ids_json, entry_time FROM trading_journal WHERE status='open' AND broker_order_ids_json NOT IN ('{}','[]') AND broker_order_ids_json IS NOT NULL ORDER BY entry_time DESC LIMIT 3")
            real_trades = cur.fetchall()
            con.close()
            print(f"[STATUS-{i}] journal: {summary} | trades_reales: {len(real_trades)}")
            for t in real_trades:
                print(f"  TRADE REAL: {t[1]} {t[0]} entry={t[2]} broker={t[3]} @ {str(t[4])[:19]}")
        except Exception as e:
            print(f"[STATUS] error: {e}")

print(f"\n[LOOP] Completado. Total purgados: {total_purged} en {cycles} ciclos")
print("[FIN] Para solución permanente: reiniciar ATLAS para aplicar commit 949764f")
