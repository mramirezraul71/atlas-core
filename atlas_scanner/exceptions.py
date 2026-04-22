from __future__ import annotations


class AtlasScannerError(Exception):
    """Base exception for atlas_scanner errors."""


class ScannerInputError(AtlasScannerError):
    """Raised when scanner input payloads are invalid."""


class ScannerProviderError(AtlasScannerError):
    """Raised when provider integration fails."""


class ScannerScoringError(AtlasScannerError):
    """Raised when scoring execution fails."""

