from __future__ import annotations

from dataclasses import dataclass
from datetime import date
from typing import Any


@dataclass(frozen=True)
class QueryResult:
    symbol: str
    status: str
    rows: int
    first_date: str | None
    last_date: str | None
    detail: str


def _classify_error(exc: Exception) -> str:
    text = str(exc).lower()
    if "api key" in text or "credential" in text or "token" in text or "unauthorized" in text:
        return "missing_credentials"
    if "parameter" in text or "invalid" in text or "provider" in text or "endpoint" in text:
        return "parameter_or_endpoint_error"
    if "connection" in text or "timeout" in text or "network" in text:
        return "connection_or_runtime_error"
    return "connection_or_runtime_error"


def _extract_dates(df: Any) -> tuple[str | None, str | None]:
    if getattr(df, "empty", True):
        return None, None
    if "date" in getattr(df, "columns", []):
        series = df["date"]
    else:
        series = getattr(df, "index", [])
    try:
        first = str(series.min())
        last = str(series.max())
    except Exception:
        return None, None
    return first, last


def _query_history(obb: Any, symbol: str, start_date: date, end_date: date) -> QueryResult:
    try:
        response = obb.equity.price.historical(
            symbol,
            start_date=start_date.isoformat(),
            end_date=end_date.isoformat(),
        )
        df = response.to_df() if hasattr(response, "to_df") else None
        if df is None:
            return QueryResult(
                symbol=symbol,
                status="parameter_or_endpoint_error",
                rows=0,
                first_date=None,
                last_date=None,
                detail="Response object has no to_df()",
            )
        rows = int(getattr(df, "shape", [0])[0])
        first_date, last_date = _extract_dates(df)
        status = "ok" if rows > 0 else "empty"
        return QueryResult(
            symbol=symbol,
            status=status,
            rows=rows,
            first_date=first_date,
            last_date=last_date,
            detail="query_success",
        )
    except Exception as exc:
        return QueryResult(
            symbol=symbol,
            status=_classify_error(exc),
            rows=0,
            first_date=None,
            last_date=None,
            detail=str(exc),
        )


def main() -> int:
    start_date = date(2025, 1, 2)
    end_date = date(2025, 1, 10)
    print(f"[info] date_range={start_date.isoformat()}..{end_date.isoformat()}")

    try:
        from openbb import obb  # type: ignore
    except Exception as exc:
        print("[error] openbb_import_failed")
        print(f"[error] status=no_backend detail={exc}")
        return 2

    print("[info] openbb_import_ok")

    results = [
        _query_history(obb, "SPY", start_date=start_date, end_date=end_date),
        _query_history(obb, "QQQ", start_date=start_date, end_date=end_date),
    ]
    for result in results:
        print(
            "[result] "
            f"symbol={result.symbol} status={result.status} rows={result.rows} "
            f"first_date={result.first_date} last_date={result.last_date}"
        )
        if result.status not in {"ok", "empty"}:
            print(f"[detail] symbol={result.symbol} detail={result.detail}")

    if any(result.status == "ok" for result in results):
        print("[summary] OpenBB backend is returning real rows for at least one symbol.")
        return 0

    print("[summary] OpenBB backend did not return real rows.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
