from __future__ import annotations


def test_dynamic_risk_destructive_command_requires_approval():
    from modules.humanoid.governance.dynamic_risk import assess_shell_command

    a = assess_shell_command("rd /s /q C:\\", cwd="C:\\ATLAS_PUSH")
    assert a.requires_approval is True
    assert a.risk in ("high", "critical")


def test_dynamic_risk_safe_command_no_approval():
    from modules.humanoid.governance.dynamic_risk import assess_shell_command

    a = assess_shell_command("pytest -q", cwd="C:\\ATLAS_PUSH")
    assert a.requires_approval is False


def test_dynamic_risk_screen_destructive_requires_approval():
    from modules.humanoid.governance.dynamic_risk import assess_action

    a = assess_action("screen_act_destructive", {"confirm_text": "Eliminar"})
    assert a.requires_approval is True
    assert a.risk in ("high", "critical")

