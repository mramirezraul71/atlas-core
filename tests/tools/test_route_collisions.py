from __future__ import annotations

from collections import defaultdict


def test_no_duplicate_http_routes():
    # Import late so module-level route dedupe has already run.
    from atlas_adapter.atlas_http_api import app

    collisions: dict[tuple[str, tuple[str, ...]], list[str]] = defaultdict(list)

    for route in app.routes:
        methods = getattr(route, "methods", None)
        if not methods:
            continue
        http_methods = tuple(
            sorted(m for m in methods if m not in {"HEAD", "OPTIONS"})
        )
        if not http_methods:
            continue
        endpoint = getattr(route.endpoint, "__name__", str(route.endpoint))
        key = (route.path, http_methods)
        collisions[key].append(endpoint)

    duplicates = {k: v for k, v in collisions.items() if len(v) > 1}
    assert not duplicates, f"Duplicate HTTP routes detected: {duplicates}"
