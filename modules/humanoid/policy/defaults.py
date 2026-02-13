"""Default policy: role=owner allowlist; update-apply and kill-process DENY."""
from __future__ import annotations

import os
from typing import List

from .models import PolicyRule


def _env_list(name: str, default: str) -> List[str]:
    v = os.getenv(name, default)
    return [x.strip() for x in (v or "").split(",") if x.strip()]


def get_default_policies() -> List[PolicyRule]:
    """Base policy: owner can update-check, llm, plan, read in allowed paths, exec with prefix allowlist; apply/kill DENY."""
    cmd_prefixes = _env_list(
        "POLICY_ALLOWED_COMMAND_PREFIXES",
        "pip,python,git,uvicorn,pytest,where,Get-Command,Invoke-RestMethod,curl.exe",
    )
    return [
        PolicyRule(id="owner_update_check", scope="update", condition={"role": "owner", "action": "check"}, action="allow"),
        PolicyRule(id="owner_llm", scope="llm", condition={"role": "owner"}, action="allow"),
        PolicyRule(id="owner_plan", scope="autonomy", condition={"role": "owner", "action": "plan"}, action="allow"),
        PolicyRule(id="owner_read_file", scope="hands", condition={"role": "owner", "action": "read_file"}, action="allow"),
        PolicyRule(id="owner_exec_prefix", scope="hands", condition={"role": "owner", "action": "exec_command", "prefix_allowed": True}, action="allow"),
        PolicyRule(id="deny_update_apply", scope="update", condition={"action": "apply"}, action="deny", meta={"description": "update-apply denied by default"}),
        PolicyRule(id="deny_kill_process", scope="hands", condition={"action": "kill_process"}, action="deny", meta={"description": "kill-process denied by default"}),
        PolicyRule(id="healing_restart_scheduler", scope="healing", condition={"role": "system", "action": "restart_scheduler"}, action="allow"),
        # Memory: read/write/export allow for owner; delete denied by default
        PolicyRule(id="owner_memory_read", scope="memory", condition={"role": "owner", "action": "memory_read"}, action="allow"),
        PolicyRule(id="owner_memory_write", scope="memory", condition={"role": "owner", "action": "memory_write"}, action="allow"),
        PolicyRule(id="owner_memory_export", scope="memory", condition={"role": "owner", "action": "memory_export"}, action="allow"),
        PolicyRule(id="deny_memory_delete", scope="memory", condition={"action": "memory_delete"}, action="deny", meta={"description": "memory_delete denied by default"}),
        PolicyRule(id="owner_ci_autofix", scope="ci", condition={"role": "owner", "action": "ci_autofix"}, action="allow"),
        PolicyRule(id="deny_deploy_apply", scope="deploy", condition={"action": "deploy_apply"}, action="deny", meta={"description": "deploy_apply denied by default"}),
        PolicyRule(id="owner_deploy_apply", scope="deploy", condition={"role": "owner", "action": "deploy_apply"}, action="allow"),
        PolicyRule(id="deny_canary_config", scope="deploy", condition={"action": "canary_config"}, action="deny", meta={"description": "canary_config denied by default"}),
        PolicyRule(id="owner_canary_config", scope="deploy", condition={"role": "owner", "action": "canary_config"}, action="allow"),
        PolicyRule(id="deny_remote_execute", scope="cluster", condition={"action": "remote_execute"}, action="deny", meta={"description": "remote_execute denied by default"}),
        PolicyRule(id="owner_remote_execute", scope="cluster", condition={"role": "owner", "action": "remote_execute"}, action="allow"),
        PolicyRule(id="hq_remote_execute", scope="cluster", condition={"role": "hq", "action": "remote_execute"}, action="allow"),
        PolicyRule(id="deny_gateway_check", scope="gateway", condition={"action": "gateway_check"}, action="deny", meta={"description": "gateway_check denied by default"}),
        PolicyRule(id="owner_gateway_check", scope="gateway", condition={"role": "owner", "action": "gateway_check"}, action="allow"),
        PolicyRule(id="deny_gateway_bootstrap", scope="gateway", condition={"action": "gateway_bootstrap"}, action="deny", meta={"description": "gateway_bootstrap denied by default"}),
        PolicyRule(id="owner_gateway_bootstrap", scope="gateway", condition={"role": "owner", "action": "gateway_bootstrap"}, action="allow"),
        PolicyRule(id="deny_gateway_set_mode", scope="gateway", condition={"action": "gateway_set_mode"}, action="deny", meta={"description": "gateway_set_mode denied by default"}),
        PolicyRule(id="owner_gateway_set_mode", scope="gateway", condition={"role": "owner", "action": "gateway_set_mode"}, action="allow"),
        PolicyRule(id="ga_autorun", scope="ga", condition={"role": "system", "action": "ga_autorun"}, action="allow"),
    ]


def get_allowed_command_prefixes() -> List[str]:
    return _env_list(
        "POLICY_ALLOWED_COMMAND_PREFIXES",
        "pip,python,git,uvicorn,pytest,where,Get-Command,Invoke-RestMethod,curl.exe",
    )
