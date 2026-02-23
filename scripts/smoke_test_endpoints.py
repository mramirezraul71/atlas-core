"""Smoke test for critical Atlas API endpoints."""
import json
import sys
import urllib.request

BASE = "http://127.0.0.1:8791"

ENDPOINTS = [
    ("GET", "/health", 200),
    ("GET", "/audit/tail?n=3", 200),
    ("GET", "/api/autonomy/status", 200),
    ("GET", "/watchdog/status", 200),
    ("GET", "/v4", 200),
]


def test_endpoint(method: str, path: str, expected_status: int) -> dict:
    """Make a request to the given endpoint and return result dict."""
    url = f"{BASE}{path}"
    try:
        req = urllib.request.Request(url, method=method)
        with urllib.request.urlopen(req, timeout=10) as resp:
            status = resp.status
            ok = status == expected_status
            return {"path": path, "status": status, "ok": ok}
    except urllib.error.HTTPError as e:
        return {"path": path, "status": e.code, "ok": e.code == expected_status}
    except Exception as e:
        return {"path": path, "status": 0, "ok": False, "error": str(e)[:120]}


def main():
    """Run all endpoint tests and exit with 1 if any fail."""
    results = []
    passed = 0
    failed = 0

    for method, path, expected in ENDPOINTS:
        r = test_endpoint(method, path, expected)
        results.append(r)
        if r["ok"]:
            passed += 1
            print(f"  OK   {path} -> {r['status']}")
        else:
            failed += 1
            err = r.get("error", "")
            print(f"  FAIL {path} -> {r['status']} {err}")

    print(f"\nResults: {passed}/{len(ENDPOINTS)} passed, {failed} failed")

    if failed > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
