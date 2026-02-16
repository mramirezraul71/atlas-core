#!/usr/bin/env python3
"""
Check SQLite Database Integrity
Verifica la integridad de las bases de datos SQLite del proyecto.
"""
import glob
import sqlite3
import sys
from pathlib import Path

def main():
    root = Path(__file__).resolve().parent.parent
    dbs = list(glob.glob(str(root / "**/*.db"), recursive=True))[:15]
    
    print(f"Checking {len(dbs)} databases...")
    
    ok_count = 0
    fail_count = 0
    
    for db_path in dbs:
        try:
            conn = sqlite3.connect(db_path)
            result = conn.execute("PRAGMA integrity_check").fetchone()[0]
            conn.close()
            
            rel_path = str(Path(db_path).relative_to(root))
            if result == "ok":
                print(f"  OK: {rel_path}")
                ok_count += 1
            else:
                print(f"  FAIL: {rel_path} - {result}")
                fail_count += 1
        except Exception as e:
            rel_path = str(Path(db_path).relative_to(root)) if db_path else db_path
            print(f"  ERROR: {rel_path} - {e}")
            fail_count += 1
    
    print(f"\nSummary: {ok_count} OK, {fail_count} failed")
    return 0 if fail_count == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
