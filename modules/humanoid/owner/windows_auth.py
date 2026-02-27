"""Verificación de contraseña de usuario Windows (LogonUser). Solo Windows."""
from __future__ import annotations

import os
import sys
from typing import Tuple


def verify_windows_user(username: str, password: str) -> Tuple[bool, str]:
    """
    Verifica usuario/contraseña contra Windows.
    Devuelve (ok, error_msg).
    """
    if sys.platform != "win32":
        return False, "Solo Windows"
    if not username or not password:
        return False, "Usuario y contraseña requeridos"
    try:
        import ctypes
        import ctypes.wintypes
        token = ctypes.wintypes.HANDLE()
        domain = "."  # cuenta local
        LOGON32_LOGON_NETWORK = 3
        LOGON32_PROVIDER_DEFAULT = 0
        ok = ctypes.windll.advapi32.LogonUserW(
            username.strip(),
            domain,
            password,
            LOGON32_LOGON_NETWORK,
            LOGON32_PROVIDER_DEFAULT,
            ctypes.byref(token)
        )
        if ok:
            ctypes.windll.kernel32.CloseHandle(token)
            return True, ""
        err = ctypes.windll.kernel32.GetLastError()
        if err == 1326:  # ERROR_LOGON_FAILURE
            return False, "Usuario o contraseña incorrectos"
        return False, f"Error Windows {err}"
    except Exception as e:
        return False, str(e)


def current_windows_user() -> str:
    """Usuario Windows actual (para pre-llenar)."""
    if sys.platform != "win32":
        return ""
    return os.environ.get("USERNAME", "").strip()
