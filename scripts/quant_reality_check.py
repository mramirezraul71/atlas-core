#!/usr/bin/env python3
"""
Comprobación operativa Atlas Code-Quant — hechos comprobables, sin suposiciones.

- Carga config/atlas.env (si existe) ANTES de importar config.settings.
- Imprime JSON con: rutas de arranque del gráfico, flags de entorno, diagnóstico de visión.
- Opcional: HTTP contra servidor Quant (health + readiness).

Uso (desde la raíz del repo):
  .\\venv\\Scripts\\python.exe scripts\\quant_reality_check.py
  .\\venv\\Scripts\\python.exe scripts\\quant_reality_check.py --http http://127.0.0.1:8795
    (con --http se exige /health y /operation/readiness OK; usar --http-health-only para solo health)
  .\\venv\\Scripts\\python.exe scripts\\quant_reality_check.py --strict
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _bootstrap_path() -> None:
    root = _repo_root()
    quant = root / "atlas_code_quant"
    for p in (str(root), str(quant)):
        if p not in sys.path:
            sys.path.insert(0, p)


def _load_dotenv_first() -> None:
    root = _repo_root()
    try:
        from dotenv import load_dotenv
    except ImportError:
        return
    for name in ("config/atlas.env", "config/atlas.env.local"):
        path = root / name
        if path.is_file():
            load_dotenv(path, override=False)
            return


def _http_json(url: str, headers: dict[str, str] | None = None, timeout: float = 8.0) -> tuple[bool, Any]:
    req = urllib.request.Request(url, headers=headers or {}, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
            return True, json.loads(raw) if raw.strip() else {}
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        try:
            parsed = json.loads(body) if body.strip() else {}
        except json.JSONDecodeError:
            parsed = {"raw": body}
        return False, {"http_status": e.code, "body": parsed}
    except Exception as exc:
        return False, {"error": str(exc)}


def main() -> int:
    parser = argparse.ArgumentParser(description="Verificación real Atlas Code-Quant (env + módulos + HTTP opcional).")
    parser.add_argument(
        "--http",
        metavar="BASE",
        help="Base URL del API Quant (ej. http://127.0.0.1:8795). Llama /health y /operation/readiness; exit 1 si readiness falla.",
    )
    parser.add_argument(
        "--http-health-only",
        action="store_true",
        help="Con --http: solo exige /health OK; no fallar si readiness devuelve 404.",
    )
    parser.add_argument(
        "--api-key",
        default="",
        help="X-API-Key para readiness (por defecto QUANT_API_KEY del entorno tras cargar atlas.env).",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit 1 si QUANT_CHART_AUTO_OPEN_ENABLED=true y no hay navegador detectado para chart.",
    )
    args = parser.parse_args()

    _bootstrap_path()
    _load_dotenv_first()

    report: dict[str, Any] = {
        "repo_root": str(_repo_root()),
        "structure": {
            "atlas_code_quant": (_repo_root() / "atlas_code_quant").is_dir(),
            "config_settings": (_repo_root() / "atlas_code_quant" / "config" / "settings.py").is_file(),
        },
    }

    try:
        from config.settings import settings
        from atlas_code_quant.operations.chart_execution import ChartExecutionService
        from atlas_code_quant.operations.readiness_eval import readiness_http_body_ok
        from atlas_code_quant.operations.sensor_vision import SensorVisionService
    except Exception as exc:
        report["import_error"] = str(exc)
        print(json.dumps(report, indent=2, ensure_ascii=True))
        return 1

    svc = ChartExecutionService()
    chart_status = svc.status()
    vision = SensorVisionService()
    vision_status = vision.status(fast=False)

    report["quant_settings"] = {
        "api_port": int(getattr(settings, "api_port", 0)),
        "chart_auto_open_enabled": bool(getattr(settings, "chart_auto_open_enabled", False)),
        "chart_verify_after_open": bool(getattr(settings, "chart_verify_after_open", True)),
        "chart_open_cooldown_sec": int(getattr(settings, "chart_open_cooldown_sec", 0)),
    }
    report["chart_execution_status"] = chart_status
    report["vision_status_local"] = vision_status

    report["behavior_contract"] = {
        "chart_browser_opens_only_when": [
            "Variable QUANT_CHART_AUTO_OPEN_ENABLED=true",
            "Una orden u operación construye chart_plan.targets (URLs)",
            "OperationCenter._build_visual_entry_gate llama ensure_chart_mission",
            "Opcional en arranque API: QUANT_STARTUP_CHART_WARMUP=true (build_selector_chart_plan)",
            "Navegador resuelto (chart_launcher._chrome_path) y verificación tasklist si está activa",
        ],
        "camera_governed_by": [
            "Estado en data/operation/sensor_vision_state.json (proveedor)",
            "POST /operation/vision/provider para insta360 | desktop_capture | direct_nexus | off",
            "Opcional: QUANT_DEFAULT_VISION_PROVIDER al arrancar el API (persiste en sensor_vision_state.json)",
            "camera_plan.required en la orden para exigir proveedor listo en el gate visual",
            "StrategySelector alinea camera_plan.provider con el proveedor persistido",
        ],
        "atlas_quant_core_camera_start": "Solo si ejecutas ATLASQuantCore.setup() / live loop, no al levantar solo uvicorn.",
        "readiness_http": (
            "GET /operation/readiness = modo fast (liviano); ok y data.ready son semánticos. "
            "Diagnóstico profundo: GET /operation/readiness/diagnostic."
        ),
    }

    if args.http:
        base = args.http.rstrip("/")
        ok_h, health = _http_json(f"{base}/health")
        report["http_health_ok"] = ok_h
        report["http_health"] = health

        key = (args.api_key or getattr(settings, "api_key", "") or "").strip()
        headers = {"X-API-Key": key, "Accept": "application/json"} if key else {"Accept": "application/json"}
        paths_try = ("/operation/readiness", "/api/v2/quant/operation/readiness")
        ok_r = False
        readiness: Any = {}
        report["http_readiness_urls_tried"] = [f"{base}{p}" for p in paths_try]
        for path in paths_try:
            ok_r, readiness = _http_json(f"{base}{path}", headers=headers)
            if ok_r:
                report["http_readiness_url_used"] = f"{base}{path}"
                break
        sem_ok = bool(ok_r and readiness_http_body_ok(readiness))
        report["http_readiness_http_200"] = bool(ok_r)
        report["http_readiness_semantic_ok"] = sem_ok
        report["http_readiness_ok"] = sem_ok
        report["http_readiness"] = readiness
        if not ok_r:
            report["http_readiness_hint"] = (
                "404 suele indicar proceso uvicorn antiguo: reinicia Quant desde el repo actual "
                "o usa la ruta documentada en api/main.py."
            )
        elif ok_r and not sem_ok:
            report["http_readiness_hint"] = (
                "HTTP 200 pero ok=false o data.ready=false en JSON; el sistema no cumple el gate operativo mínimo "
                "(revisa reasons_not_ready en data)."
            )

    print(json.dumps(report, indent=2, ensure_ascii=True))

    if args.strict:
        auto = bool(report["quant_settings"]["chart_auto_open_enabled"])
        browser_ok = bool(chart_status.get("browser_available"))
        if auto and not browser_ok:
            print(
                "\n[strict] FALLA: QUANT_CHART_AUTO_OPEN_ENABLED=true pero browser_available=false "
                "(instala Chrome/Edge o revisa chart_launcher._chrome_path).",
                file=sys.stderr,
            )
            return 1

    if args.http and not report.get("http_health_ok"):
        return 1
    if args.http and not args.http_health_only and not report.get("http_readiness_ok"):
        print(
            "\n[FALLA] Readiness con --http: se exige HTTP 200 y cuerpo con ok=true y data.ready=true "
            "(gate operativo). Ver http_readiness_hint en JSON. --http-health-only omite esta comprobación.",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
