"""Configuración centralizada e internacionalización (i18n)."""
from __future__ import annotations

from .i18n import (SUPPORTED_LOCALES, get_all_strings, get_locale, get_text,
                   set_locale)

__all__ = [
    "get_text",
    "get_locale",
    "set_locale",
    "get_all_strings",
    "SUPPORTED_LOCALES",
]
