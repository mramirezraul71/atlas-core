"""risk.kill_switch — file-based kill switch (F19).

F1 scaffold consolidado en F19 con la implementación real:
``FileKillSwitch``, ``KillSwitchStatus``,
``load_kill_switch_path_from_env``.
"""

from atlas_code_quant.risk.kill_switch.file_switch import (
    FileKillSwitch,
    KillSwitchStatus,
    load_kill_switch_path_from_env,
)

__all__ = [
    "FileKillSwitch",
    "KillSwitchStatus",
    "load_kill_switch_path_from_env",
]
