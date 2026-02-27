from __future__ import annotations

from typing import Dict, List, Optional

from .models import POT
from .pots.git_safe_sync import get_pot as get_git_safe_sync_pot
from .pots.system_doctor import get_pot as get_system_doctor_pot
from .pots.services_ports_check import get_pot as get_services_ports_check_pot
from .pots.quality_visits_audit import get_pot as get_quality_visits_audit_pot
from .pots.vault_snapshot_backup import get_pot as get_vault_snapshot_backup_pot
from .pots.humanoid_arch_audit import get_pot as get_humanoid_arch_audit_pot


def list_pots() -> List[Dict]:
    pots = [
        get_git_safe_sync_pot(),
        get_system_doctor_pot(),
        get_services_ports_check_pot(),
        get_quality_visits_audit_pot(),
        get_vault_snapshot_backup_pot(),
        get_humanoid_arch_audit_pot(),
    ]
    return [
        {
            "id": p.id,
            "name": p.name,
            "severity": p.severity.value if hasattr(p.severity, "value") else str(p.severity),
            "description": p.description,
            "tags": list(p.tags),
            "rules": list(p.rules),
        }
        for p in pots
    ]


def get_pot(pot_id: str) -> Optional[POT]:
    pid = (pot_id or "").strip()
    if pid == "git_safe_sync":
        return get_git_safe_sync_pot()
    if pid == "system_doctor":
        return get_system_doctor_pot()
    if pid == "services_ports_check":
        return get_services_ports_check_pot()
    if pid == "quality_visits_audit":
        return get_quality_visits_audit_pot()
    if pid == "vault_snapshot_backup":
        return get_vault_snapshot_backup_pot()
    if pid == "humanoid_arch_audit":
        return get_humanoid_arch_audit_pot()
    return None

