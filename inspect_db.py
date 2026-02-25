#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import sqlite3
from pathlib import Path

db_path = Path("C:\\ATLAS_PUSH\\data\\autonomy_tasks.db")

if not db_path.exists():
    print(f"BD no encontrada: {db_path}")
    exit(1)

conn = sqlite3.connect(str(db_path))
cursor = conn.cursor()

# Obtener todas las tablas
cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
tables = cursor.fetchall()

print("=" * 80)
print("TABLAS EN LA BASE DE DATOS")
print("=" * 80)

for (table_name,) in tables:
    print(f"\nTABLA: {table_name}")
    print("-" * 80)

    # Esquema
    cursor.execute(f"PRAGMA table_info({table_name});")
    columns = cursor.fetchall()
    print("Columnas:")
    for col in columns:
        print(f"  - {col[1]} ({col[2]})")

    # Contar registros
    cursor.execute(f"SELECT COUNT(*) FROM {table_name};")
    count = cursor.fetchone()[0]
    print(f"\nRegistros: {count}")

    # Mostrar primeros 3 registros
    if count > 0:
        cursor.execute(f"SELECT * FROM {table_name} LIMIT 3;")
        rows = cursor.fetchall()
        print("\nPrimeros registros:")
        for row in rows:
            print(f"  {row}")

conn.close()
print("\n" + "=" * 80)
