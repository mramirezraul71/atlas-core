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
    # UI (v4 should be canonical at /ui)
    ("GET", "/ui", 200),
    ("GET", "/ui/static/style.css", 200),
    ("GET", "/ui/static/app.js", 200),
    # Back-compat: /v4 should still work (redirects to /ui)
    ("GET", "/v4", 200),
    ("GET", "/v4/static/app.js", 200),
    # Thread utilities
    ("POST", "/conversation/thread/new", 200),
    # Key ops endpoints used by v4 modules
    ("GET", "/approvals/pending?limit=1", 200),
    ("GET", "/api/kernel/event-bus/history?limit=1", 200),
    ("GET", "/api/healing/history?limit=1", 200),
    ("GET", "/api/cognitive-memory/lifelog/status", 200),
    ("GET", "/ans/comms/status", 200),
    ("GET", "/ans/voice/status", 200),
]


def test_endpoint(method: str, path: str, expected_status: int) -> dict:
    """Make a request to the given endpoint and return result dict."""
    url = f"{BASE}{path}"
    try:
        headers = {}
        data = None
        if method.upper() == "POST":
            data = json.dumps({}).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
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
