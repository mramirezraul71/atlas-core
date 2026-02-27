#!/usr/bin/env python3
"""
Diagnóstico y activación ANS.
Ejecuta: forzar GROWTH, ejecutar ANS, mostrar resultados.
"""
from __future__ import annotations

import os
import sys

# Cargar atlas.env
from pathlib import Path
BASE = str(Path(__file__).resolve().parent.parent)
env_path = os.path.join(BASE, "config", "atlas.env")
if os.path.isfile(env_path):
    from dotenv import load_dotenv
    load_dotenv(env_path, override=True)
else:
    print("[!] atlas.env no encontrado en", env_path)

# Asegurar path de módulos
sys.path.insert(0, BASE)

def main():
    print("=== ANS Diagnóstico ===\n")

    # 1. Env
    system_mode = os.getenv("SYSTEM_MODE", "")
    gov_mode = os.getenv("GOVERNANCE_MODE", "")
    ans_force = os.getenv("ANS_GROWTH_FORCE", "")
    print(f"ENV: SYSTEM_MODE={system_mode!r} GOVERNANCE_MODE={gov_mode!r} ANS_GROWTH_FORCE={ans_force!r}")

    # 2. Governance actual
    try:
        from modules.humanoid.governance.state import get_mode, set_mode
        actual = get_mode()
        print(f"Governance actual (DB): {actual}")

        if actual != "growth":
            print(">> Forzando modo GROWTH...")
            ok = set_mode("growth", reason="ans_diagnostico", actor="script")
            print(f"  set_mode(growth) = {ok}")
    except Exception as e:
        print(f"[X] Error governance: {e}")

    # 3. Bypass
    try:
        from modules.humanoid.ans.engine import _ans_bypass_governance
        bypass = _ans_bypass_governance()
        print(f"ANS bypass governance: {bypass}")
    except Exception as e:
        print(f"[X] Error bypass: {e}")

    # 4. Ejecutar ANS
    print("\n>> Ejecutando run_ans_cycle...")
    try:
        from modules.humanoid.ans.engine import run_ans_cycle
        r = run_ans_cycle(mode="auto", timeout_sec=60)
        print(f"  ok={r.get('ok')} issues={r.get('issues_count', 0)} actions={len(r.get('actions_taken', []))} ms={r.get('ms')}")
        for a in r.get("actions_taken", [])[:10]:
            print(f"    - {a.get('heal_id')}: ok={a.get('ok')} msg={a.get('message', '')[:60]}")
    except Exception as e:
        print(f"[X] Error ANS: {e}")
        import traceback
        traceback.print_exc()

    # 5. Health score (run_health_verbose)
    try:
        from modules.humanoid.deploy.healthcheck import run_health_verbose
        h = run_health_verbose(base_url=None, active_port=int(os.getenv("ACTIVE_PORT", "8791") or 8791))
        ch = h.get("checks", {})
        print(f"\nHealth score: {h.get('score', 0)}/100 (ok={h.get('ok')}) | ans_open_incidents={ch.get('ans_open_incidents', '?')}")
        for k in ("api_up", "memory_writable", "audit_writable", "scheduler_alive", "llm_reachable"):
            v = (ch or {}).get(k)
            if isinstance(v, dict):
                ok = v.get("ok", v.get("writable", v.get("running", v.get("reachable", "?"))))
            else:
                ok = v
            print(f"  {k}: {ok}")
    except Exception as e:
        print(f"\n[X] Error health: {e}")

    # 6. Incidentes abiertos
    try:
        from modules.humanoid.ans.incident import get_incidents
        open_inc = [i for i in get_incidents(status=None, limit=10) if i.get("status") == "open"]
        print(f"\nIncidentes abiertos: {len(open_inc)}")
        for inc in open_inc[:5]:
            acts = inc.get("actions_taken", [])
            acts_str = ", ".join(str(a.get("message", ""))[:40] for a in acts) or "—"
            print(f"  - {inc.get('check_id')}: {inc.get('message', '')[:50]} | acciones: {acts_str}")
    except Exception as e:
        print(f"[X] Error incidentes: {e}")

    print("\n=== Fin ===")

if __name__ == "__main__":
    main()
