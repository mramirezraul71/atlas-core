#!/usr/bin/env python3
"""
Script: Repo Scanner para POT auto_update_full
Ejecuta análisis determinístico del repositorio.
"""
import json
import sys
from pathlib import Path

# Agregar root al path
BASE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE))

def main():
    """Ejecutar repo scanner y mostrar resultados."""
    try:
        from modules.humanoid.ci.scanner import scan_repo
        result = scan_repo(scope="repo", max_items=20)
        
        print("=== REPO SCANNER ===")
        print(f"OK: {result.get('ok', False)}")
        print(f"Hallazgos: {len(result.get('findings', []))}")
        print()
        
        for f in result.get('findings', [])[:10]:
            kind = f.get('kind', 'unknown')
            path = f.get('path', '')
            detail = f.get('detail', '')
            print(f"  [{kind}] {path}: {detail}")
        
        # Retornar JSON para captura
        print()
        print("JSON_OUTPUT:", json.dumps(result))
        
        return 0 if result.get('ok') else 1
        
    except ImportError as e:
        print(f"Error importando scanner: {e}")
        print("JSON_OUTPUT:", json.dumps({"ok": False, "error": str(e)}))
        return 1
    except Exception as e:
        print(f"Error ejecutando scanner: {e}")
        print("JSON_OUTPUT:", json.dumps({"ok": False, "error": str(e)}))
        return 1


if __name__ == "__main__":
    sys.exit(main())
