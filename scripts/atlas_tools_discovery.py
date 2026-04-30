#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import time
import urllib.request
from dataclasses import dataclass
from typing import Any


@dataclass
class Candidate:
    id: str
    name: str
    category: str
    multitask_value: str
    install_method: str
    install_target: str
    network_probe: str
    docs_url: str


CATALOG: list[Candidate] = [
    Candidate("task", "Taskwarrior", "orchestration", "cola local de tareas para automatizaciÃ³n", "winget", "Task.Task", "https://taskwarrior.org", "https://taskwarrior.org/docs/"),
    Candidate("tmux", "tmux", "terminal", "multiplexaciÃ³n de sesiones para ejecuciÃ³n concurrente", "winget", "arndawg.tmux-windows", "https://github.com/tmux/tmux", "https://github.com/tmux/tmux/wiki"),
    Candidate("yt-dlp", "yt-dlp", "media", "captura de fuentes multimedia para pipelines", "pip", "yt-dlp", "https://pypi.org/pypi/yt-dlp/json", "https://github.com/yt-dlp/yt-dlp"),
    Candidate("httpie", "HTTPie", "network", "validaciÃ³n rÃ¡pida de APIs y flujos externos", "winget", "HTTPie.HTTPie", "https://httpie.io", "https://httpie.io/docs"),
    Candidate("nmap", "Nmap", "network", "mapeo de disponibilidad de red para agentes", "winget", "Insecure.Nmap", "https://nmap.org", "https://nmap.org/book/man.html"),
    Candidate("jq", "jq", "data", "procesamiento JSON para cadenas multitarea", "winget", "jqlang.jq", "https://jqlang.github.io/jq/", "https://jqlang.github.io/jq/manual/"),
    Candidate("ruff", "ruff", "quality", "lint/format ultrarrÃ¡pido para automatizaciones de cÃ³digo", "pip", "ruff", "https://pypi.org/pypi/ruff/json", "https://docs.astral.sh/ruff/"),
    Candidate("uv", "uv", "runtime", "gestiÃ³n rÃ¡pida de entornos y dependencias Python", "winget", "astral-sh.uv", "https://astral.sh", "https://docs.astral.sh/uv/"),
    Candidate("playwright", "Playwright", "browser", "automatizaciÃ³n web robusta para agentes", "npm", "playwright", "https://registry.npmjs.org/playwright", "https://playwright.dev/"),
]


def _probe(url: str, timeout_sec: float = 5.0) -> tuple[bool, int | None, str]:
    t0 = time.perf_counter()
    try:
        req = urllib.request.Request(url, method="GET", headers={"User-Agent": "atlas-tools-discovery/1.0"})
        with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
            _ = resp.read(1024)
            latency = int((time.perf_counter() - t0) * 1000)
            return True, latency, ""
    except Exception as e:
        return False, None, str(e)[:160]


def _extract_version(method: str, payload: Any) -> str | None:
    try:
        if method == "pip":
            return str((payload or {}).get("info", {}).get("version") or "").strip() or None
        if method == "npm":
            return str((payload or {}).get("dist-tags", {}).get("latest") or "").strip() or None
    except Exception:
        return None
    return None


def _latest_version(c: Candidate) -> str | None:
    if c.install_method not in ("pip", "npm"):
        return None
    try:
        req = urllib.request.Request(c.network_probe, method="GET", headers={"User-Agent": "atlas-tools-discovery/1.0"})
        with urllib.request.urlopen(req, timeout=6) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
        payload = json.loads(raw)
        return _extract_version(c.install_method, payload)
    except Exception:
        return None


def main() -> int:
    rows: list[dict[str, Any]] = []
    ok_count = 0
    for c in CATALOG:
        available, latency_ms, err = _probe(c.network_probe)
        latest = _latest_version(c)
        if available:
            ok_count += 1
        score = 50
        if available:
            score += 30
        if latest:
            score += 10
        if c.category in ("orchestration", "browser", "network"):
            score += 10
        score = max(0, min(100, score))
        rows.append(
            {
                "id": c.id,
                "name": c.name,
                "category": c.category,
                "multitask_value": c.multitask_value,
                "install_method": c.install_method,
                "install_target": c.install_target,
                "network_probe": c.network_probe,
                "docs_url": c.docs_url,
                "network_available": available,
                "latency_ms": latency_ms,
                "latest_version": latest,
                "score": score,
                "status": "ready" if available else "unreachable",
                "network_error": err or None,
            }
        )

    rows.sort(key=lambda x: (x.get("network_available") is not True, -int(x.get("score") or 0), str(x.get("name") or "")))
    out = {
        "ok": True,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "summary": {
            "total": len(rows),
            "available": ok_count,
            "unreachable": len(rows) - ok_count,
        },
        "candidates": rows,
    }
    print(json.dumps(out, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


