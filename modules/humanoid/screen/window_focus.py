from __future__ import annotations

import os
import sys
from typing import Optional


def get_active_window_title() -> str:
    """
    Windows-only (ctypes): obtiene el título de la ventana en primer plano.
    Best-effort: si falla o no es Windows, retorna "".
    """
    if os.name != "nt":
        return ""
    try:
        import ctypes
        from ctypes import wintypes

        user32 = ctypes.WinDLL("user32", use_last_error=True)
        GetForegroundWindow = user32.GetForegroundWindow
        GetWindowTextLengthW = user32.GetWindowTextLengthW
        GetWindowTextW = user32.GetWindowTextW

        hwnd = GetForegroundWindow()
        if not hwnd:
            return ""
        length = GetWindowTextLengthW(hwnd)
        if length <= 0:
            # Algunas ventanas devuelven 0; intentamos buffer mínimo
            length = 256
        buf = ctypes.create_unicode_buffer(length + 1)
        GetWindowTextW(hwnd, buf, length + 1)
        return (buf.value or "").strip()
    except Exception:
        return ""


def get_active_window_process_path() -> str:
    """
    Windows-only (ctypes): obtiene la ruta del ejecutable del proceso dueño de la ventana foreground.
    Best-effort: si falla, retorna "".
    """
    if os.name != "nt":
        return ""
    try:
        import ctypes

        user32 = ctypes.WinDLL("user32", use_last_error=True)
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

        GetForegroundWindow = user32.GetForegroundWindow
        GetWindowThreadProcessId = user32.GetWindowThreadProcessId

        OpenProcess = kernel32.OpenProcess
        CloseHandle = kernel32.CloseHandle

        # QueryFullProcessImageNameW está en kernel32
        QueryFullProcessImageNameW = kernel32.QueryFullProcessImageNameW
        QueryFullProcessImageNameW.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_wchar_p, ctypes.POINTER(ctypes.c_uint32)]
        QueryFullProcessImageNameW.restype = ctypes.c_int

        hwnd = GetForegroundWindow()
        if not hwnd:
            return ""
        pid = ctypes.c_uint32(0)
        GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        if not pid.value:
            return ""
        PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
        hproc = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid.value)
        if not hproc:
            return ""
        try:
            buf_len = ctypes.c_uint32(260)
            buf = ctypes.create_unicode_buffer(buf_len.value)
            ok = QueryFullProcessImageNameW(hproc, 0, buf, ctypes.byref(buf_len))
            if not ok:
                return ""
            return (buf.value or "").strip()
        finally:
            try:
                CloseHandle(hproc)
            except Exception:
                pass
    except Exception:
        return ""


def active_window_process_matches(expected_exe_substring: str) -> bool:
    exp = (expected_exe_substring or "").strip().lower()
    if not exp:
        return True
    p = get_active_window_process_path().lower()
    return exp in p if p else False


def active_window_matches(expected_substring: str) -> bool:
    exp = (expected_substring or "").strip().lower()
    if not exp:
        return True
    title = get_active_window_title().lower()
    return exp in title if title else False

