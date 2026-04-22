from __future__ import annotations

from typing import Any, Tuple


class BaseProvider:
    provider_name: str = "base"

    def get_symbols(self) -> Tuple[str, ...]:
        raise NotImplementedError

    def get_quote(self, symbol: str) -> Any:
        _ = symbol
        raise NotImplementedError

