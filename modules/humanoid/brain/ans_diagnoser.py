"""Cerebro ANS: usa el self-model para diagnosticar y razonar sobre el estado del sistema."""
from __future__ import annotations

from typing import Any, Dict, List, Optional


def diagnose_with_self_model(incidents: List[Dict], actions: List[Dict], manifest: Optional[Dict] = None) -> Dict[str, Any]:
    """
    El cerebro analiza incidentes usando autoconocimiento.
    Retorna: diagnóstico estructurado, causas probables, heals disponibles/no disponibles.
    """
    if manifest is None:
        try:
            from modules.humanoid.self_model import get_manifest
            manifest = get_manifest()
        except Exception:
            manifest = {}

    ns = manifest.get("anatomy", {}).get("nervous_system", {})
    checks = {c["id"]: c for c in ns.get("checks", [])}
    heals = {h["id"]: h for h in ns.get("heals", [])}
    check_to_heal = manifest.get("dependency_graph", {}).get("check_to_heal", {})

    diagnosis = {
        "summary": "",
        "incidents_by_check": {},
        "heals_available": [],
        "heals_blocked": [],
        "root_causes": [],
        "recommendations": [],
    }

    for inc in incidents:
        cid = inc.get("check_id", "?")
        actions_taken = inc.get("actions_taken", [])
        suggested = check_to_heal.get(cid, [])

        if cid not in diagnosis["incidents_by_check"]:
            diagnosis["incidents_by_check"][cid] = {"incident": inc, "suggested_heals": suggested, "tried": [], "blocked": []}

        for a in actions_taken:
            hid = a.get("heal_id", "")
            if hid and hid != "(omitido)":
                diagnosis["incidents_by_check"][cid]["tried"].append(a)
            elif "omitido" in str(a.get("message", "")):
                diagnosis["incidents_by_check"][cid]["blocked"].append(a.get("message", ""))

        for hid in suggested:
            if hid in heals and heals[hid].get("safe"):
                if not any(t.get("heal_id") == hid for t in diagnosis["incidents_by_check"][cid]["tried"]):
                    diagnosis["heals_available"].append({"check": cid, "heal": hid})
            else:
                diagnosis["heals_blocked"].append({"check": cid, "heal": hid, "reason": "no safe" if hid not in heals else "not in SAFE_HEALS"})

    # Root causes: checks que fallan sin heal efectivo
    for cid, data in diagnosis["incidents_by_check"].items():
        if data["blocked"] and not data["tried"]:
            diagnosis["root_causes"].append({
                "check": cid,
                "message": data["incident"].get("message", ""),
                "blocked_reasons": data["blocked"],
            })

    # Recommendations
    if diagnosis["heals_blocked"]:
        diagnosis["recommendations"].append("Algunos heals están bloqueados. Revisar GOVERNANCE_MODE=growth o SAFE_HEALS.")
    if diagnosis["root_causes"]:
        diagnosis["recommendations"].append("Incidentes sin heal disponible: considerar añadir heals o corrección manual.")
    if not diagnosis["root_causes"] and not diagnosis["heals_blocked"] and incidents:
        diagnosis["recommendations"].append("Heals ejecutados. Verificar que incidentes se resolvieron.")

    issues = len(incidents)
    acted = len(actions)
    diagnosis["summary"] = f"{issues} incidentes, {acted} acciones. Causas raíz: {len(diagnosis['root_causes'])}."

    return diagnosis
