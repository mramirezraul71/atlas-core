# -*- coding: utf-8 -*-
import subprocess
from core.logger import log

def _run(cmd):
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            shell=True
        )
        return result.stdout.strip(), result.stderr.strip()
    except Exception as e:
        return "", str(e)

def run():
    log("RAULI DOCTOR INICIADO")

    checks = {
        "flutter_version": ["flutter", "--version"],
        "flutter_doctor": ["flutter", "doctor", "-v"],
        "dart_version": ["dart", "--version"],
        "where_flutter": ["where", "flutter"],
        "where_dart": ["where", "dart"],
        "where_python": ["where", "python"]
    }

    for name, cmd in checks.items():
        log(f"--- {name.upper()} ---")
        out, err = _run(cmd)
        if out:
            log(out)
        if err:
            log("ERROR:")
            log(err)

    log("RAULI DOCTOR FINALIZADO")
