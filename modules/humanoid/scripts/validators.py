"""Validators: check scripts have headers, error handling, structured output."""
from __future__ import annotations

import re
from typing import List, Tuple


def has_ps_header(content: str) -> bool:
    """PowerShell: param block or comment header."""
    stripped = content.strip()
    if stripped.startswith("#") and ("param" in content.lower() or "Requires" in content.lower()):
        return True
    if re.search(r"^\s*param\s*\(", content, re.MULTILINE | re.IGNORECASE):
        return True
    return stripped.startswith("#")


def has_python_header(content: str) -> bool:
    """Python: shebang or docstring."""
    stripped = content.strip()
    if stripped.startswith("#!"):
        return True
    if stripped.startswith('"""') or stripped.startswith("'''"):
        return True
    return bool(stripped.startswith("#"))


def has_error_handling_ps(content: str) -> bool:
    """PS: try/catch or $ErrorActionPreference."""
    return "try" in content and "catch" in content or "$ErrorActionPreference" in content


def has_error_handling_python(content: str) -> bool:
    """Python: try/except."""
    return "try:" in content and "except" in content


def validate_script(kind: str, content: str) -> Tuple[bool, List[str]]:
    """Return (valid, list of missing requirements)."""
    missing = []
    if kind.lower() == "powershell":
        if not has_ps_header(content):
            missing.append("header (param block or comment)")
        if not has_error_handling_ps(content):
            missing.append("error handling (try/catch or $ErrorActionPreference)")
    elif kind.lower() == "python":
        if not has_python_header(content):
            missing.append("header (shebang or docstring)")
        if not has_error_handling_python(content):
            missing.append("error handling (try/except)")
    return (len(missing) == 0, missing)
