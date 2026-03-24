from __future__ import annotations

import argparse
import json
import os
import re
import sqlite3
import subprocess
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from importlib import metadata as importlib_metadata
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "logs"
LOG_FILE = LOG_DIR / "snapshot_safe_diagnostic.log"
CACHE_FILE = LOG_DIR / "atlas_tools_watchdog_cache.json"
REGISTRY_FILE = ROOT / "atlas_master_registry.json"
ACTUATORS_LOCK = ROOT / "tools" / "atlas_actuators" / "package-lock.json"
COMMS_DB = Path(
    (os.getenv("ATLAS_COMMS_DB_PATH") or str(ROOT / "logs" / "atlas_comms_hub.sqlite")).strip()
)
LATEST_TTL_SECONDS = 15 * 60
PROTECTED_BRANCHES = {"main", "master", "release", "prod", "production"}

LOG_DIR.mkdir(parents=True, exist_ok=True)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _append_log(line: str) -> None:
    ts = _utc_now()
    with LOG_FILE.open("a", encoding="utf-8") as f:
        f.write(f"{ts} {line}\n")


def _run_cmd(cmd: list[str], timeout: int = 8, cwd: Optional[Path] = None) -> Tuple[bool, str]:
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd or ROOT),
            capture_output=True,
            text=True,
            timeout=timeout,
            shell=False,
        )
        out = (proc.stdout or proc.stderr or "").strip()
        return proc.returncode == 0, out
    except Exception as ex:
        return False, str(ex)


def _load_cache() -> Dict[str, Any]:
    if not CACHE_FILE.exists():
        return {"latest": {}, "logged_upgrades": {}}
    try:
        return json.loads(CACHE_FILE.read_text(encoding="utf-8"))
    except Exception:
        return {"latest": {}, "logged_upgrades": {}}


def _save_cache(data: Dict[str, Any]) -> None:
    CACHE_FILE.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")


def _http_json(url: str, timeout: int = 8) -> Tuple[bool, Dict[str, Any], str]:
    req = urllib.request.Request(
        url=url,
        headers={
            "Accept": "application/json",
            "User-Agent": "ATLAS-Tools-Watchdog/1.0",
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
        parsed = json.loads(raw)
        return True, parsed if isinstance(parsed, dict) else {"data": parsed}, ""
    except urllib.error.HTTPError as ex:
        return False, {}, f"http_{ex.code}"
    except Exception as ex:
        return False, {}, str(ex)


def _extract_version(text: str) -> str:
    if not text:
        return ""
    m = re.search(r"(\d+\.\d+\.\d+(?:[-+._a-zA-Z0-9]*)?)", text)
    if m:
        return m.group(1).lstrip("vV")
    m2 = re.search(r"(\d+\.\d+)", text)
    return m2.group(1).lstrip("vV") if m2 else text.strip()


def _norm_version_tuple(raw: str) -> Tuple[int, ...]:
    if not raw:
        return tuple()
    nums = [int(x) for x in re.findall(r"\d+", raw)]
    return tuple(nums[:4]) if nums else tuple()


def _is_newer(latest: str, local: str) -> bool:
    t_latest = _norm_version_tuple(latest)
    t_local = _norm_version_tuple(local)
    if not t_latest or not t_local:
        return False
    return t_latest > t_local


def _get_pkg_version(pkg_name: str) -> str:
    venv_py = ROOT / ".venv" / "Scripts" / "python.exe"
    if venv_py.exists():
        ok, out = _run_cmd(
            [
                str(venv_py),
                "-c",
                (
                    "import importlib.metadata as m; "
                    f"print(m.version('{pkg_name}'))"
                ),
            ],
            timeout=8,
        )
        if ok and out:
            return out.splitlines()[-1].strip()

    try:
        return importlib_metadata.version(pkg_name)
    except Exception:
        pass
    return ""


def _winget_installed(package_id: str) -> Tuple[bool, str]:
    if not package_id:
        return False, ""
    ok, out = _run_cmd(["winget", "list", "--id", package_id, "--source", "winget"], timeout=20)
    text = (out or "").strip()
    if not ok:
        return False, text
    low = text.lower()
    if package_id.lower() not in low:
        return False, text
    return True, text


def _get_puppeteer_version() -> str:
    if not ACTUATORS_LOCK.exists():
        return ""
    try:
        data = json.loads(ACTUATORS_LOCK.read_text(encoding="utf-8", errors="ignore"))
        packages = data.get("packages") or {}
        direct = packages.get("node_modules/puppeteer") or {}
        if isinstance(direct, dict) and direct.get("version"):
            return str(direct.get("version"))
        deps = (data.get("dependencies") or {}).get("puppeteer") or {}
        if isinstance(deps, dict) and deps.get("version"):
            return str(deps.get("version"))
        return ""
    except Exception:
        return ""


def _queue_pending() -> int:
    if not COMMS_DB.exists():
        return 0
    try:
        conn = sqlite3.connect(str(COMMS_DB), timeout=5)
        try:
            cur = conn.execute(
                "SELECT COUNT(1) FROM comms_offline_queue WHERE status='pending';"
            )
            row = cur.fetchone()
            return int(row[0]) if row else 0
        finally:
            conn.close()
    except Exception:
        return 0


def _tool_status(ok: bool, warning: bool = False) -> str:
    if ok and not warning:
        return "ok"
    if ok and warning:
        return "warn"
    return "error"


def _git_context() -> Dict[str, Any]:
    ok, out = _run_cmd(["git", "rev-parse", "--abbrev-ref", "HEAD"], timeout=8)
    branch = out.splitlines()[-1].strip() if ok and out else "unknown"
    allow_raw = (os.getenv("ATLAS_ALLOW_TOOL_UPDATE_ON_PROTECTED") or "0").strip().lower()
    allow_protected = allow_raw not in {"0", "false", "no", "off", ""}
    return {
        "branch": branch,
        "is_protected": branch in PROTECTED_BRANCHES,
        "protected_branches": sorted(PROTECTED_BRANCHES),
        "allow_protected_updates": allow_protected,
    }


def _local_inventory() -> Dict[str, Dict[str, Any]]:
    ok_ollama, out_ollama = _run_cmd(["ollama", "--version"])
    ok_cf, out_cf = _run_cmd(["cloudflared", "--version"])
    ok_git, out_git = _run_cmd(["git", "--version"])
    ok_node, out_node = _run_cmd(["node", "--version"])
    ok_tailscale, out_tailscale = _run_cmd(["tailscale", "version"])
    ok_httpie, out_httpie = _run_cmd(["http", "--version"])
    ok_nmap, out_nmap = _run_cmd(["nmap", "--version"])
    ok_task, out_task = _run_cmd(["task", "--version"])
    ok_ruff, out_ruff = _run_cmd(["ruff", "--version"])
    ok_ytdlp, out_ytdlp = _run_cmd(["yt-dlp", "--version"])
    ok_jq, out_jq = _run_cmd(["jq", "--version"])
    ok_tmux, out_tmux = _run_cmd(["tmux", "-V"])
    ok_uv, out_uv = _run_cmd(["uv", "--version"])
    wg_httpie_ok, wg_httpie_out = _winget_installed("HTTPie.HTTPie")
    wg_nmap_ok, wg_nmap_out = _winget_installed("Insecure.Nmap")
    wg_task_ok, wg_task_out = _winget_installed("Task.Task")
    wg_tmux_ok, wg_tmux_out = _winget_installed("arndawg.tmux-windows")

    cloudflared_proc = False
    try:
        r = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq cloudflared.exe"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        cloudflared_proc = "cloudflared.exe" in (r.stdout or "").lower()
    except Exception:
        cloudflared_proc = False

    core = (os.getenv("ATLAS_CENTRAL_CORE") or "").strip()
    approvals = (os.getenv("APPROVALS_CHAIN_SECRET") or "").strip()
    clawd_token_ready = bool(core or approvals)

    sqlite_ver = sqlite3.sqlite_version
    ccxt_ver = _get_pkg_version("ccxt")
    playwright_ver = _get_pkg_version("playwright")
    ruff_ver = _get_pkg_version("ruff")
    ytdlp_ver = _get_pkg_version("yt-dlp")
    puppeteer_ver = _get_puppeteer_version()

    # ── Trading / ML stack (atlas_code_quant) ─────────────────────────
    pandas_ver      = _get_pkg_version("pandas")
    numpy_ver       = _get_pkg_version("numpy")
    scipy_ver       = _get_pkg_version("scipy")
    sklearn_ver     = _get_pkg_version("scikit-learn")
    xgboost_ver     = _get_pkg_version("xgboost")
    lightgbm_ver    = _get_pkg_version("lightgbm")
    statsmodels_ver = _get_pkg_version("statsmodels")
    ta_ver          = _get_pkg_version("ta")
    yfinance_ver    = _get_pkg_version("yfinance")
    optuna_ver      = _get_pkg_version("optuna")
    sb3_ver         = _get_pkg_version("stable-baselines3")
    backtrader_ver  = _get_pkg_version("backtrader")
    gymnasium_ver   = _get_pkg_version("gymnasium")
    easyocr_ver     = _get_pkg_version("easyocr")

    # ── Core framework ─────────────────────────────────────────────────
    fastapi_ver    = _get_pkg_version("fastapi")
    uvicorn_ver    = _get_pkg_version("uvicorn")
    httpx_ver      = _get_pkg_version("httpx")
    sqlalchemy_ver = _get_pkg_version("SQLAlchemy")
    pydantic_ver   = _get_pkg_version("pydantic")

    # ── Software: Prometheus ───────────────────────────────────────────
    prometheus_ver = ""
    ok_prometheus  = False
    for _pd in [ROOT / "tools" / "prometheus", Path("C:/prometheus"), Path("C:/Program Files/prometheus")]:
        _exe = _pd / "prometheus.exe"
        if _exe.exists():
            _ok_p, _out_p = _run_cmd([str(_exe), "--version"], timeout=6)
            if _out_p:
                prometheus_ver = _extract_version(_out_p)
                ok_prometheus  = True
            break

    # ── Software: Grafana ──────────────────────────────────────────────
    ok_grafana, out_grafana = _run_cmd(["grafana-server", "--version"], timeout=6)
    grafana_ver = _extract_version(out_grafana) if ok_grafana else ""
    wg_grafana_ok = False
    if not grafana_ver:
        wg_grafana_ok, wg_grafana_out = _winget_installed("GrafanaLabs.Grafana")
        grafana_ver = _extract_version(wg_grafana_out) if wg_grafana_ok else ""
        ok_grafana  = ok_grafana or wg_grafana_ok

    # ── Software: Docker ───────────────────────────────────────────────
    ok_docker, out_docker = _run_cmd(["docker", "--version"], timeout=8)
    docker_ver = _extract_version(out_docker) if ok_docker else ""

    # ── Software: Python interpreter ───────────────────────────────────
    ok_python, out_python = _run_cmd(["python", "--version"], timeout=5)
    python_ver = _extract_version(out_python) if ok_python else ""

    pending = _queue_pending()
    queue_warn = pending > 0

    tools: Dict[str, Dict[str, Any]] = {
        "ollama": {
            "id": "ollama",
            "name": "Ollama Runtime",
            "category": "runtime",
            "critical": True,
            "version": _extract_version(out_ollama) if ok_ollama else "",
            "status": _tool_status(ok_ollama),
            "health": "online" if ok_ollama else "not_detected",
            "details": out_ollama if not ok_ollama else "",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool ollama",
        },
        "cloudflared": {
            "id": "cloudflared",
            "name": "Cloudflare Tunnel",
            "category": "network",
            "critical": True,
            "version": _extract_version(out_cf) if ok_cf else "",
            "status": _tool_status(ok_cf, warning=ok_cf and not cloudflared_proc),
            "health": "running" if cloudflared_proc else ("installed" if ok_cf else "not_detected"),
            "details": out_cf if out_cf else "",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool cloudflared",
        },
        "git": {
            "id": "git",
            "name": "Git CLI",
            "category": "devops",
            "critical": True,
            "version": _extract_version(out_git) if ok_git else "",
            "status": _tool_status(ok_git),
            "health": "online" if ok_git else "not_detected",
            "details": out_git if out_git else "",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool git",
        },
        "node": {
            "id": "node",
            "name": "Node.js",
            "category": "runtime",
            "critical": True,
            "version": _extract_version(out_node.lstrip("vV")) if ok_node else "",
            "status": _tool_status(ok_node),
            "health": "online" if ok_node else "not_detected",
            "details": out_node if out_node else "",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool node",
        },
        "ccxt": {
            "id": "ccxt",
            "name": "CCXT (Trading)",
            "category": "dependency",
            "critical": False,
            "version": ccxt_ver,
            "status": _tool_status(bool(ccxt_ver)),
            "health": "ready" if ccxt_ver else "missing",
            "details": "",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool ccxt",
        },
        "playwright_py": {
            "id": "playwright_py",
            "name": "Playwright Python",
            "category": "dependency",
            "critical": False,
            "version": playwright_ver,
            "status": _tool_status(bool(playwright_ver)),
            "health": "ready" if playwright_ver else "missing",
            "details": "",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool playwright_py",
        },
        "puppeteer": {
            "id": "puppeteer",
            "name": "Puppeteer Node",
            "category": "dependency",
            "critical": False,
            "version": puppeteer_ver,
            "status": _tool_status(bool(puppeteer_ver)),
            "health": "ready" if puppeteer_ver else "missing",
            "details": "tools/atlas_actuators",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool puppeteer",
        },
        "sqlite": {
            "id": "sqlite",
            "name": "SQLite Local Store",
            "category": "resilience",
            "critical": False,
            "version": sqlite_ver,
            "status": _tool_status(True, warning=queue_warn),
            "health": "queue_pending" if queue_warn else "ready",
            "details": f"offline_queue_pending={pending}",
            "update_script": "",
        },
        "tailscale": {
            "id": "tailscale",
            "name": "Tailscale Backup Tunnel",
            "category": "network",
            "critical": True,
            "version": _extract_version(out_tailscale) if ok_tailscale else "",
            "status": _tool_status(ok_tailscale),
            "health": "ready" if ok_tailscale else "not_detected",
            "details": out_tailscale if out_tailscale else "",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool tailscale",
        },
        "clawdbot_core": {
            "id": "clawdbot_core",
            "name": "ClawdBOT Core Token",
            "category": "security",
            "critical": True,
            "version": "configured" if clawd_token_ready else "missing",
            "status": _tool_status(clawd_token_ready),
            "health": "ready" if clawd_token_ready else "missing_token",
            "details": "ATLAS_CENTRAL_CORE / APPROVALS_CHAIN_SECRET",
            "update_script": "",
        },
        "httpie": {
            "id": "httpie",
            "name": "HTTPie",
            "category": "network",
            "critical": False,
            "version": _extract_version(out_httpie if ok_httpie else wg_httpie_out),
            "status": _tool_status(ok_httpie or wg_httpie_ok, warning=(wg_httpie_ok and not ok_httpie)),
            "health": "ready" if ok_httpie else ("installed_no_path" if wg_httpie_ok else "not_detected"),
            "details": (
                out_httpie
                if ok_httpie and out_httpie
                else ("installed via winget (PATH pendiente)" if wg_httpie_ok else (out_httpie if out_httpie else ""))
            ),
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool httpie",
        },
        "nmap": {
            "id": "nmap",
            "name": "Nmap",
            "category": "network",
            "critical": False,
            "version": _extract_version(out_nmap if ok_nmap else wg_nmap_out),
            "status": _tool_status(ok_nmap or wg_nmap_ok, warning=(wg_nmap_ok and not ok_nmap)),
            "health": "ready" if ok_nmap else ("installed_no_path" if wg_nmap_ok else "not_detected"),
            "details": out_nmap if out_nmap else wg_nmap_out,
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool nmap",
        },
        "task": {
            "id": "task",
            "name": "Taskwarrior",
            "category": "orchestration",
            "critical": False,
            "version": (_extract_version(out_task) if ok_task else (_extract_version(wg_task_out) if wg_task_ok else "")),
            "status": _tool_status(ok_task or wg_task_ok, warning=(wg_task_ok and not ok_task)),
            "health": "ready" if ok_task else ("installed_no_path" if wg_task_ok else "not_detected"),
            "details": (
                out_task
                if ok_task and out_task
                else ("installed via winget (PATH pendiente)" if wg_task_ok else (out_task if out_task else ""))
            ),
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool task",
        },
        "ruff": {
            "id": "ruff",
            "name": "ruff",
            "category": "quality",
            "critical": False,
            "version": ruff_ver or (_extract_version(out_ruff) if ok_ruff else ""),
            "status": _tool_status(bool(ruff_ver or ok_ruff)),
            "health": "ready" if (ruff_ver or ok_ruff) else "not_detected",
            "details": (
                (f"venv_package={ruff_ver}" + (f"; cmd={out_ruff.strip()}" if ok_ruff and out_ruff else ""))
                if ruff_ver
                else (out_ruff if out_ruff else "")
            ),
            "update_script": "",
        },
        "yt-dlp": {
            "id": "yt-dlp",
            "name": "yt-dlp",
            "category": "media",
            "critical": False,
            "version": ytdlp_ver or (_extract_version(out_ytdlp) if ok_ytdlp else ""),
            "status": _tool_status(bool(ytdlp_ver or ok_ytdlp)),
            "health": "ready" if (ytdlp_ver or ok_ytdlp) else "not_detected",
            "details": (
                (f"venv_package={ytdlp_ver}" + (f"; cmd={out_ytdlp.strip()}" if ok_ytdlp and out_ytdlp else ""))
                if ytdlp_ver
                else (out_ytdlp if out_ytdlp else "")
            ),
            "update_script": "",
        },
        "jq": {
            "id": "jq",
            "name": "jq",
            "category": "data",
            "critical": False,
            "version": _extract_version(out_jq) if ok_jq else "",
            "status": _tool_status(ok_jq),
            "health": "ready" if ok_jq else "not_detected",
            "details": out_jq if out_jq else "",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool jq",
        },
        "tmux": {
            "id": "tmux",
            "name": "tmux",
            "category": "terminal",
            "critical": False,
            "version": (_extract_version(out_tmux) if ok_tmux else (_extract_version(wg_tmux_out) if wg_tmux_ok else "")),
            "status": _tool_status(ok_tmux or wg_tmux_ok, warning=(wg_tmux_ok and not ok_tmux)),
            "health": "ready" if ok_tmux else ("installed_no_path" if wg_tmux_ok else "not_detected"),
            "details": (
                out_tmux
                if ok_tmux and out_tmux
                else ("installed via winget (PATH pendiente)" if wg_tmux_ok else (out_tmux if out_tmux else ""))
            ),
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool tmux",
        },
        "uv": {
            "id": "uv",
            "name": "uv",
            "category": "runtime",
            "critical": False,
            "version": _extract_version(out_uv) if ok_uv else "",
            "status": _tool_status(ok_uv),
            "health": "ready" if ok_uv else "not_detected",
            "details": out_uv if out_uv else "",
            "update_script": "",
        },
        # ── Trading / ML stack ──────────────────────────────────────────
        "pandas": {
            "id": "pandas",
            "name": "pandas (Data)",
            "category": "trading",
            "critical": True,
            "version": pandas_ver,
            "status": _tool_status(bool(pandas_ver)),
            "health": "ready" if pandas_ver else "missing",
            "details": "Análisis de series temporales de mercado",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool pandas",
        },
        "numpy": {
            "id": "numpy",
            "name": "NumPy (Cómputo)",
            "category": "trading",
            "critical": True,
            "version": numpy_ver,
            "status": _tool_status(bool(numpy_ver)),
            "health": "ready" if numpy_ver else "missing",
            "details": "Vectores, arrays y álgebra numérica",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool numpy",
        },
        "scipy": {
            "id": "scipy",
            "name": "SciPy (Estadística)",
            "category": "trading",
            "critical": True,
            "version": scipy_ver,
            "status": _tool_status(bool(scipy_ver)),
            "health": "ready" if scipy_ver else "missing",
            "details": "Pricing Black-Scholes + estadística quant",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool scipy",
        },
        "scikit-learn": {
            "id": "scikit-learn",
            "name": "scikit-learn (ML)",
            "category": "trading",
            "critical": True,
            "version": sklearn_ver,
            "status": _tool_status(bool(sklearn_ver)),
            "health": "ready" if sklearn_ver else "missing",
            "details": "Clasificadores, scalers y pipelines ML",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool scikit-learn",
        },
        "xgboost": {
            "id": "xgboost",
            "name": "XGBoost (ML Trading)",
            "category": "trading",
            "critical": True,
            "version": xgboost_ver,
            "status": _tool_status(bool(xgboost_ver)),
            "health": "ready" if xgboost_ver else "missing",
            "details": "Signal ranker — fallback a RandomForest",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool xgboost",
        },
        "lightgbm": {
            "id": "lightgbm",
            "name": "LightGBM (ML Trading)",
            "category": "trading",
            "critical": True,
            "version": lightgbm_ver,
            "status": _tool_status(bool(lightgbm_ver)),
            "health": "ready" if lightgbm_ver else "missing",
            "details": "Signal ranker principal — 19 features",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool lightgbm",
        },
        "statsmodels": {
            "id": "statsmodels",
            "name": "statsmodels (Series)",
            "category": "trading",
            "critical": False,
            "version": statsmodels_ver,
            "status": _tool_status(bool(statsmodels_ver)),
            "health": "ready" if statsmodels_ver else "missing",
            "details": "ARIMA, cointegración y tests estadísticos",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool statsmodels",
        },
        "ta": {
            "id": "ta",
            "name": "TA (Indicadores Técnicos)",
            "category": "trading",
            "critical": True,
            "version": ta_ver,
            "status": _tool_status(bool(ta_ver)),
            "health": "ready" if ta_ver else "missing",
            "details": "RSI, MACD, Bollinger, ATR, EMA",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool ta",
        },
        "yfinance": {
            "id": "yfinance",
            "name": "yfinance (Market Data)",
            "category": "trading",
            "critical": True,
            "version": yfinance_ver,
            "status": _tool_status(bool(yfinance_ver)),
            "health": "ready" if yfinance_ver else "missing",
            "details": "OHLCV histórico + datos de opciones Yahoo",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool yfinance",
        },
        "optuna": {
            "id": "optuna",
            "name": "Optuna (Hypertuning)",
            "category": "trading",
            "critical": False,
            "version": optuna_ver,
            "status": _tool_status(bool(optuna_ver)),
            "health": "ready" if optuna_ver else "missing",
            "details": "Optimización de hiperparámetros RL cada 4-24h",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool optuna",
        },
        "stable-baselines3": {
            "id": "stable-baselines3",
            "name": "Stable-Baselines3 (RL)",
            "category": "trading",
            "critical": False,
            "version": sb3_ver,
            "status": _tool_status(bool(sb3_ver)),
            "health": "ready" if sb3_ver else "missing",
            "details": "PPO/DQN/A2C — agentes RL de trading",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool stable-baselines3",
        },
        "backtrader": {
            "id": "backtrader",
            "name": "Backtrader (Backtesting)",
            "category": "trading",
            "critical": False,
            "version": backtrader_ver,
            "status": _tool_status(bool(backtrader_ver)),
            "health": "ready" if backtrader_ver else "missing",
            "details": "Simulación histórica de estrategias",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool backtrader",
        },
        "gymnasium": {
            "id": "gymnasium",
            "name": "Gymnasium (RL Env)",
            "category": "trading",
            "critical": False,
            "version": gymnasium_ver,
            "status": _tool_status(bool(gymnasium_ver)),
            "health": "ready" if gymnasium_ver else "missing",
            "details": "Entornos RL compatibles OpenAI Gym",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool gymnasium",
        },
        "easyocr": {
            "id": "easyocr",
            "name": "EasyOCR (Visual Trading)",
            "category": "trading",
            "critical": False,
            "version": easyocr_ver,
            "status": _tool_status(bool(easyocr_ver)),
            "health": "ready" if easyocr_ver else "missing",
            "details": "Extracción de precios desde pantalla",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool easyocr",
        },
        # ── Core framework ──────────────────────────────────────────────
        "fastapi": {
            "id": "fastapi",
            "name": "FastAPI",
            "category": "framework",
            "critical": True,
            "version": fastapi_ver,
            "status": _tool_status(bool(fastapi_ver)),
            "health": "ready" if fastapi_ver else "missing",
            "details": "API principal Atlas + Quant (8791/8795)",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool fastapi",
        },
        "uvicorn": {
            "id": "uvicorn",
            "name": "Uvicorn (ASGI)",
            "category": "framework",
            "critical": True,
            "version": uvicorn_ver,
            "status": _tool_status(bool(uvicorn_ver)),
            "health": "ready" if uvicorn_ver else "missing",
            "details": "Servidor ASGI — hot reload activo",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool uvicorn",
        },
        "httpx": {
            "id": "httpx",
            "name": "httpx (HTTP async)",
            "category": "framework",
            "critical": True,
            "version": httpx_ver,
            "status": _tool_status(bool(httpx_ver)),
            "health": "ready" if httpx_ver else "missing",
            "details": "Cliente HTTP async — model router + trading",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool httpx",
        },
        "sqlalchemy": {
            "id": "sqlalchemy",
            "name": "SQLAlchemy (ORM)",
            "category": "framework",
            "critical": True,
            "version": sqlalchemy_ver,
            "status": _tool_status(bool(sqlalchemy_ver)),
            "health": "ready" if sqlalchemy_ver else "missing",
            "details": "ORM — journal de trades + episodic memory",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool sqlalchemy",
        },
        "pydantic": {
            "id": "pydantic",
            "name": "Pydantic (Schemas)",
            "category": "framework",
            "critical": True,
            "version": pydantic_ver,
            "status": _tool_status(bool(pydantic_ver)),
            "health": "ready" if pydantic_ver else "missing",
            "details": "Validación de schemas API y config",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool pydantic",
        },
        # ── Software externo ────────────────────────────────────────────
        "prometheus": {
            "id": "prometheus",
            "name": "Prometheus (Métricas)",
            "category": "software",
            "critical": False,
            "version": prometheus_ver,
            "status": _tool_status(ok_prometheus, warning=ok_prometheus and not prometheus_ver),
            "health": "ready" if ok_prometheus else "not_detected",
            "details": "tools/prometheus/ — métricas Atlas + Quant",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool prometheus",
        },
        "grafana": {
            "id": "grafana",
            "name": "Grafana (Dashboard)",
            "category": "software",
            "critical": False,
            "version": grafana_ver,
            "status": _tool_status(ok_grafana, warning=wg_grafana_ok and not ok_grafana),
            "health": "ready" if ok_grafana else "not_detected",
            "details": "Puerto 3002 — dashboards métricas y trading",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool grafana",
        },
        "docker": {
            "id": "docker",
            "name": "Docker",
            "category": "software",
            "critical": False,
            "version": docker_ver,
            "status": _tool_status(ok_docker),
            "health": "ready" if ok_docker else "not_detected",
            "details": "Contenedores — servicios auxiliares",
            "update_script": "scripts\\atlas_tool_update.ps1 -Tool docker",
        },
        "python": {
            "id": "python",
            "name": "Python",
            "category": "software",
            "critical": True,
            "version": python_ver,
            "status": _tool_status(ok_python),
            "health": "ready" if ok_python else "not_detected",
            "details": "Intérprete principal — venv en C:\\ATLAS_PUSH\\venv",
            "update_script": "",
        },
    }
    return tools


def _latest_fetchers() -> Dict[str, Any]:
    return {
        # ── Herramientas sistema ──────────────────────────────────────────
        "ollama":      lambda: _github_release_latest("ollama", "ollama"),
        "cloudflared": lambda: _github_release_latest("cloudflare", "cloudflared"),
        "git":         lambda: _github_release_latest("git-for-windows", "git"),
        "node":        _node_latest,
        "tailscale":   lambda: _github_release_latest("tailscale", "tailscale"),
        "jq":          lambda: _github_release_latest("jqdashboard", "jq"),
        "task":        lambda: _github_release_latest("GothenburgBitFactory", "taskwarrior"),
        "tmux":        lambda: _github_release_latest("tmux", "tmux"),
        # ── PyPI — herramientas existentes ────────────────────────────────
        "ccxt":        lambda: _pypi_latest("ccxt"),
        "playwright_py": lambda: _pypi_latest("playwright"),
        "ruff":        lambda: _pypi_latest("ruff"),
        "yt-dlp":      lambda: _pypi_latest("yt-dlp"),
        "uv":          lambda: _pypi_latest("uv"),
        "httpie":      lambda: _pypi_latest("httpie"),
        # ── NPM ────────────────────────────────────────────────────────────
        "puppeteer":   _npm_latest_puppeteer,
        # ── PyPI — Trading / ML stack ─────────────────────────────────────
        "pandas":           lambda: _pypi_latest("pandas"),
        "numpy":            lambda: _pypi_latest("numpy"),
        "scipy":            lambda: _pypi_latest("scipy"),
        "scikit-learn":     lambda: _pypi_latest("scikit-learn"),
        "xgboost":          lambda: _pypi_latest("xgboost"),
        "lightgbm":         lambda: _pypi_latest("lightgbm"),
        "statsmodels":      lambda: _pypi_latest("statsmodels"),
        "ta":               lambda: _pypi_latest("ta"),
        "yfinance":         lambda: _pypi_latest("yfinance"),
        "optuna":           lambda: _pypi_latest("optuna"),
        "stable-baselines3": lambda: _pypi_latest("stable-baselines3"),
        "backtrader":       lambda: _pypi_latest("backtrader"),
        "gymnasium":        lambda: _pypi_latest("gymnasium"),
        "easyocr":          lambda: _pypi_latest("easyocr"),
        # ── PyPI — Core framework ─────────────────────────────────────────
        "fastapi":    lambda: _pypi_latest("fastapi"),
        "uvicorn":    lambda: _pypi_latest("uvicorn"),
        "httpx":      lambda: _pypi_latest("httpx"),
        "sqlalchemy": lambda: _pypi_latest("SQLAlchemy"),
        "pydantic":   lambda: _pypi_latest("pydantic"),
        # ── Software externo — GitHub releases ────────────────────────────
        "prometheus": lambda: _github_release_latest("prometheus", "prometheus"),
        "grafana":    lambda: _github_release_latest("grafana", "grafana"),
        "docker":     lambda: _github_release_latest("moby", "moby"),
    }


def _github_release_latest(owner: str, repo: str) -> str:
    ok, data, _err = _http_json(f"https://api.github.com/repos/{owner}/{repo}/releases/latest")
    if not ok:
        return ""
    return _extract_version(str(data.get("tag_name") or data.get("name") or ""))


def _pypi_latest(pkg: str) -> str:
    ok, data, _err = _http_json(f"https://pypi.org/pypi/{pkg}/json")
    if not ok:
        return ""
    info = data.get("info") or {}
    return _extract_version(str(info.get("version") or ""))


def _npm_latest_puppeteer() -> str:
    ok, data, _err = _http_json("https://registry.npmjs.org/puppeteer/latest")
    if not ok:
        return ""
    return _extract_version(str(data.get("version") or ""))


def _node_latest() -> str:
    req = urllib.request.Request(
        url="https://nodejs.org/dist/index.json",
        headers={"User-Agent": "ATLAS-Tools-Watchdog/1.0"},
    )
    try:
        with urllib.request.urlopen(req, timeout=8) as resp:
            data = json.loads(resp.read().decode("utf-8", errors="replace"))
        if isinstance(data, list) and data:
            return _extract_version(str(data[0].get("version") or ""))
        return ""
    except Exception:
        return ""


def _nmap_latest() -> str:
    """Fetch latest Nmap version from GitHub API or fallback to empty."""
    # Try releases endpoint (most projects use this)
    ok, data, _err = _http_json("https://api.github.com/repos/nmap/nmap/releases/latest")
    if ok and data:
        version = _extract_version(str(data.get("tag_name") or data.get("name") or ""))
        if version:
            return version
    
    # Fallback: try to get from tags
    ok2, data2, _err2 = _http_json("https://api.github.com/repos/nmap/nmap/tags?per_page=1")
    if ok2 and isinstance(data2, list) and data2:
        version = _extract_version(str(data2[0].get("name") or ""))
        if version:
            return version
    
    return ""


def _resolve_latest_versions(
    tool_map: Dict[str, Dict[str, Any]], cache: Dict[str, Any], force: bool
) -> Dict[str, str]:
    latest_cache = cache.setdefault("latest", {})
    fetchers = _latest_fetchers()
    now = int(time.time())
    latest: Dict[str, str] = {}

    for tool_id, tool in tool_map.items():
        if tool_id not in fetchers:
            latest[tool_id] = ""
            continue
        cached = latest_cache.get(tool_id) or {}
        cache_age_ok = (
            isinstance(cached, dict)
            and not force
            and int(cached.get("ts") or 0) > 0
            and (now - int(cached.get("ts") or 0)) <= LATEST_TTL_SECONDS
            and bool(cached.get("version"))
        )
        if cache_age_ok:
            latest[tool_id] = str(cached.get("version") or "")
            continue
        version = ""
        try:
            version = str(fetchers[tool_id]() or "")
        except Exception:
            version = ""
        latest_cache[tool_id] = {"version": version, "ts": now}
        latest[tool_id] = version
    return latest


def _build_registry(force: bool = False) -> Dict[str, Any]:
    cache = _load_cache()
    tool_map = _local_inventory()
    git_ctx = _git_context()
    latest_map = _resolve_latest_versions(tool_map, cache, force=force)
    logged = cache.setdefault("logged_upgrades", {})

    tools = []
    upgrade_ready_count = 0
    for tool_id, tool in tool_map.items():
        local = str(tool.get("version") or "")
        latest = str(latest_map.get(tool_id) or "")
        ready = bool(latest and local and _is_newer(latest, local))
        if ready:
            upgrade_ready_count += 1
            key = f"{tool_id}:{local}->{latest}"
            if logged.get(tool_id) != key:
                _append_log(f"UPGRADE_READY tool={tool_id} local={local} latest={latest}")
                logged[tool_id] = key
        tool["latest_version"] = latest
        tool["update_ready"] = ready
        if ready and tool.get("status") == "ok":
            tool["status"] = "warn"
            tool["health"] = "upgrade_ready"
        tools.append(tool)

    ok_count = sum(1 for t in tools if t.get("status") == "ok")
    warn_count = sum(1 for t in tools if t.get("status") == "warn")
    err_count = sum(1 for t in tools if t.get("status") == "error")
    pending = _queue_pending()

    registry = {
        "ok": True,
        "generated_at": _utc_now(),
        "workspace": str(ROOT),
        "scan_source": "atlas_tools_watchdog.py",
        "summary": {
            "total": len(tools),
            "ok": ok_count,
            "warn": warn_count,
            "error": err_count,
            "upgrade_ready": upgrade_ready_count,
        },
        "offline_queue": {
            "pending": pending,
            "db_path": str(COMMS_DB),
            "resync_script": "scripts\\atlas_resync_flow.ps1",
        },
        "channels": {
            "panaderia": os.getenv("ATLAS_PANADERIA_BASE_URL", "http://127.0.0.1:3001"),
            "vision": os.getenv("ATLAS_VISION_BASE_URL", "http://127.0.0.1:3000"),
            "push": os.getenv("ATLAS_PUSH_BASE_URL", "http://127.0.0.1:8791"),
        },
        "git": git_ctx,
        "security": {
            "core_token_env": bool((os.getenv("ATLAS_CENTRAL_CORE") or "").strip()),
            "approval_secret_env": bool((os.getenv("APPROVALS_CHAIN_SECRET") or "").strip()),
            "encryption_mode": "ATLAS_CENTRAL_CORE"
            if (os.getenv("ATLAS_CENTRAL_CORE") or "").strip()
            else "APPROVALS_CHAIN_SECRET"
            if (os.getenv("APPROVALS_CHAIN_SECRET") or "").strip()
            else "MACHINE_FALLBACK",
        },
        "tools": sorted(tools, key=lambda t: (str(t.get("category")), str(t.get("name")))),
    }

    REGISTRY_FILE.write_text(json.dumps(registry, indent=2, ensure_ascii=False), encoding="utf-8")
    _save_cache(cache)
    return registry


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS tools inventory and upgrade watchdog")
    parser.add_argument("--force", action="store_true", help="Force remote version refresh")
    parser.add_argument("--pretty", action="store_true", help="Pretty print json")
    args = parser.parse_args()

    data = _build_registry(force=args.force)
    if args.pretty:
        print(json.dumps(data, indent=2, ensure_ascii=False))
    else:
        print(json.dumps(data, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

