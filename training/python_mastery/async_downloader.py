from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Iterable, List, Sequence, TypeVar

T = TypeVar("T")
R = TypeVar("R")


@dataclass(frozen=True)
class BatchResult:
    ok: bool
    results: List[R]
    errors: List[str]
    cancelled: bool = False


async def gather_with_limit(
    items: Sequence[T],
    worker: Callable[[T], Awaitable[R]],
    *,
    limit: int = 5,
    timeout_s: float | None = None,
) -> BatchResult:
    """PY011: ejecutar worker(item) en paralelo con límite de concurrencia.

    Reglas esperadas (ver tests):
    - respeta `limit` (no más de N workers corriendo a la vez)
    - si se cancela, cancela pendientes y retorna cancelled=True (si logra)
    - si timeout_s, cada worker se limita con wait_for
    """
    if limit <= 0:
        raise ValueError("limit debe ser > 0")

    sem = asyncio.Semaphore(limit)
    results: List[R] = []
    errors: List[str] = []

    async def _run_one(it: T) -> None:
        async with sem:
            try:
                if timeout_s is None:
                    r = await worker(it)
                else:
                    r = await asyncio.wait_for(worker(it), timeout=timeout_s)
                results.append(r)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                errors.append(f"{type(e).__name__}: {e}")

    tasks = [asyncio.create_task(_run_one(it)) for it in items]
    try:
        await asyncio.gather(*tasks)
        return BatchResult(ok=len(errors) == 0, results=results, errors=errors, cancelled=False)
    except asyncio.CancelledError:
        # Cancelar todo lo pendiente y propagar un resultado coherente.
        for t in tasks:
            try:
                t.cancel()
            except Exception:
                pass
        try:
            await asyncio.gather(*tasks, return_exceptions=True)
        except Exception:
            pass
        return BatchResult(ok=False, results=results, errors=errors, cancelled=True)

