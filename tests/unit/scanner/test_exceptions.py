from __future__ import annotations

from atlas_scanner.exceptions import (
    AtlasScannerError,
    ScannerInputError,
    ScannerProviderError,
    ScannerScoringError,
)


def test_exception_hierarchy() -> None:
    assert issubclass(AtlasScannerError, Exception)
    assert issubclass(ScannerInputError, AtlasScannerError)
    assert issubclass(ScannerProviderError, AtlasScannerError)
    assert issubclass(ScannerScoringError, AtlasScannerError)

