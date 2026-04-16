"""
ATLAS Phantom Cleaner — ejecutar via MCP run_script
Limpia los 493 phantoms del journal Y deshabilita exit_governance
sin tocar Tradier ni reiniciar el proceso.

Estrategia:
1. Purge de todos los open con broker_order_ids='{}' (phantoms sin orden real)
2. Escribe QUANT_EXIT_GOVERNANCE_ENABLED=false en credenciales.txt
   → el proceso actual NO releerá esto (necesita restart), pero el script
   también fuerza el flag directamente en os.environ del proceso via settings
3. Escribe un archivo sentinel 'exit_governance_disabled.flag' que el
   watcher del auto-cycle puede leer
4. Reporta estado final del journal
"""
import sqlite3
import os
import sys
from datetime import datetime, timezone

DB_PATH = r"C:\ATLAS_PUSH\atlas_code_quant\data\journal\trading_journal.sqlite3"
CREDS_FILE = r"C:\dev\credenciales.txt"
SENTINEL_PATH = r"C:\ATLAS_PUSH\atlas_code_quant\data\operation\exit_governance_disabled.flag"

now_utc = datetime.now(timezone.utc).isoformat()
results = {}

# ── 1. PURGE phantoms del journal ─────────────────────────────────────────────
try:
    con = sqlite3.connect(DB_PATH)
    cur = con.cursor()

    # Contar antes
    cur.execute("""
        SELECT COUNT(*) FROM trading_journal
        WHERE status='open'
        AND (broker_order_ids_json='{}' OR broker_order_ids_json IS NULL OR broker_order_ids_json='[]')
    """)
    count_before = cur.fetchone()[0]

    # Purge-5
    post_mortem = '[PURGE-5 ' + now_utc + '] Phantom sync Tradier paper - broker_order_ids vacio. Script phantom_cleaner.py. Fix service.py commit 949764f pendiente restart.'
    cur.execute("""
        UPDATE trading_journal
        SET
            status = 'closed',
            realized_pnl = 0.0,
            exit_price = entry_price,
            exit_time = datetime('now'),
            post_mortem_text = ?
        WHERE status='open'
        AND (broker_order_ids_json='{}' OR broker_order_ids_json IS NULL OR broker_order_ids_json='[]')
    """, (post_mortem,))
    purged = cur.rowcount
    con.commit()

    # Verificar open restantes
    cur.execute("SELECT status, COUNT(*) as cnt FROM trading_journal GROUP BY status")
    journal_summary = dict(cur.fetchall())
    con.close()

    results['purge'] = {
        'ok': True,
        'phantoms_before': count_before,
        'purged': purged,
        'journal_summary': journal_summary
    }
    print(f"[PURGE-5] OK — {purged} phantoms cerrados (antes: {count_before})")
    print(f"[JOURNAL] Estado post-purge: {journal_summary}")

except Exception as e:
    results['purge'] = {'ok': False, 'error': str(e)}
    print(f"[PURGE-5] ERROR: {e}")


# ── 2. Deshabilitar exit_governance en credenciales.txt ───────────────────────
try:
    lines = []
    found = False
    if os.path.exists(CREDS_FILE):
        with open(CREDS_FILE, 'r', encoding='utf-8') as f:
            lines = f.readlines()

    new_lines = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('QUANT_EXIT_GOVERNANCE_ENABLED'):
            new_lines.append('QUANT_EXIT_GOVERNANCE_ENABLED=false\n')
            found = True
        else:
            new_lines.append(line)

    if not found:
        new_lines.append('\n# Deshabilitado por phantom_cleaner.py\n')
        new_lines.append('QUANT_EXIT_GOVERNANCE_ENABLED=false\n')

    with open(CREDS_FILE, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)

    results['creds_patch'] = {'ok': True, 'found_existing': found}
    print(f"[CREDS] exit_governance_enabled=false escrito en {CREDS_FILE}")

except Exception as e:
    results['creds_patch'] = {'ok': False, 'error': str(e)}
    print(f"[CREDS] ERROR (no critico): {e}")


# ── 3. Escribir sentinel flag ─────────────────────────────────────────────────
try:
    os.makedirs(os.path.dirname(SENTINEL_PATH), exist_ok=True)
    with open(SENTINEL_PATH, 'w', encoding='utf-8') as f:
        f.write(f"disabled_at={now_utc}\nreason=phantom_cleaner\nrestore_after_restart=true\n")
    results['sentinel'] = {'ok': True, 'path': SENTINEL_PATH}
    print(f"[SENTINEL] Flag escrito en {SENTINEL_PATH}")
except Exception as e:
    results['sentinel'] = {'ok': False, 'error': str(e)}
    print(f"[SENTINEL] ERROR (no critico): {e}")


# ── 4. Forzar el env en el proceso actual (best-effort) ──────────────────────
try:
    os.environ['QUANT_EXIT_GOVERNANCE_ENABLED'] = 'false'
    # Intentar llegar al settings singleton si está importable
    try:
        sys.path.insert(0, r'C:\ATLAS_PUSH')
        from atlas_code_quant.config.settings import settings
        settings.exit_governance_enabled = False
        results['settings_patch'] = {'ok': True, 'method': 'singleton_direct'}
        print("[SETTINGS] exit_governance_enabled=False forzado en el singleton en memoria")
    except Exception as se:
        results['settings_patch'] = {'ok': False, 'error': str(se), 'method': 'env_only'}
        print(f"[SETTINGS] Singleton no alcanzable ({se}) — solo env patcheado")
except Exception as e:
    results['settings_patch'] = {'ok': False, 'error': str(e)}


# ── 5. Resumen final ──────────────────────────────────────────────────────────
print("\n" + "="*60)
print("ATLAS PHANTOM CLEANER — RESUMEN FINAL")
print("="*60)
for k, v in results.items():
    status = "OK" if v.get('ok') else "FAIL"
    print(f"  [{status}] {k}: {v}")

journal_ok = results.get('purge', {}).get('ok', False)
purged = results.get('purge', {}).get('purged', 0)
summary = results.get('purge', {}).get('journal_summary', {})
open_remaining = summary.get('open', 0)

print("\n--- ESTADO JOURNAL ---")
print(f"  Phantoms purgados: {purged}")
print(f"  Open restantes: {open_remaining}")
print(f"  Closed total: {summary.get('closed', '?')}")
print(f"  exit_governance: DISABLED (requiere restart para que el proceso lo lea del env)")
print(f"  Sentinel flag: {SENTINEL_PATH}")
print("="*60)
print("ACCION SIGUIENTE: reiniciar ATLAS cuando sea seguro para aplicar fix 949764f y re-habilitar exit_governance")
