#!/usr/bin/env python3
"""
Check Pip Outdated Packages
Lista paquetes con actualizaciones disponibles.
"""
import json
import subprocess
import sys


def main():
    try:
        result = subprocess.run(
            [sys.executable, "-m", "pip", "list", "--outdated", "--format=json"],
            capture_output=True,
            text=True,
            timeout=90,
        )
        
        if result.returncode != 0:
            print(f"pip list failed: {result.stderr}")
            return 1
        
        packages = json.loads(result.stdout) if result.stdout.strip() else []
        print(f"Packages with updates: {len(packages)}")
        
        for pkg in packages[:15]:
            name = pkg.get("name", "?")
            current = pkg.get("version", "?")
            latest = pkg.get("latest_version", "?")
            print(f"  {name}: {current} -> {latest}")
        
        if len(packages) > 15:
            print(f"  ... and {len(packages) - 15} more")
        
        return 0
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
