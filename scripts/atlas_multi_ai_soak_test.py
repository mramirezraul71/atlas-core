#!/usr/bin/env python3
"""
Prueba prolongada del router Multi-IA (Robot :8002).

Objetivo:
- Verificar que tareas simples usan modelos rapidos.
- Verificar escalado automatico a modelos mas pesados segun especialidad.
- Validar que las respuestas se mantengan estables durante multiples rondas.

Uso:
  python scripts/atlas_multi_ai_soak_test.py
  python scripts/atlas_multi_ai_soak_test.py --rounds 4 --timeout 90
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence

import requests


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BASE_URL = "http://127.0.0.1:8002"


@dataclass(frozen=True)
class SoakCase:
    name: str
    expected_tier: str  # simple | complex
    prompt: str
    allowed_models: Sequence[str]


@dataclass
class SoakRecord:
    case_name: str
    round_idx: int
    ok: bool
    http_status: int
    model: str
    provider: str
    response_time: float
    routing_mode: str
    selected_tier: str
    complexity_level: str
    complexity_score: float
    error: str


TEST_CASES: List[SoakCase] = [
    SoakCase(
        name="general_simple",
        expected_tier="simple",
        prompt="Saluda en una sola frase y maximo 12 palabras.",
        allowed_models=("llama3.1:8b",),
    ),
    SoakCase(
        name="analysis_simple",
        expected_tier="simple",
        prompt=(
            "Resume en una linea: ATLAS monitoriza servicios y hace recuperacion "
            "automatica."
        ),
        allowed_models=("llama3.1:8b",),
    ),
    SoakCase(
        name="reasoning_simple",
        expected_tier="simple",
        prompt="Cuanto es 19*7? Devuelve solo el numero.",
        allowed_models=("llama3.1:8b", "qwen3-coder:30b"),
    ),
    SoakCase(
        name="code_simple",
        expected_tier="simple",
        prompt=(
            "Escribe una funcion Python sumar(a,b) en 3 lineas y sin explicacion."
        ),
        allowed_models=("qwen2.5-coder:7b", "deepseek-coder-v2:16b"),
    ),
    SoakCase(
        name="general_extended",
        expected_tier="simple",
        prompt=(
            "Disena un plan corto de 4 bullets para estabilizar tres servicios con "
            "healthchecks y alertas."
        ),
        allowed_models=("llama3.1:8b", "qwen3-coder:30b"),
    ),
    SoakCase(
        name="analysis_complex",
        expected_tier="complex",
        prompt=(
            "Compara en analisis detallado dos estrategias de observabilidad para un "
            "sistema local.\n"
            "Incluye diagnostico completo, evalua riesgos y cierra con una matriz de "
            "decision priorizada."
        ),
        allowed_models=("qwen3-coder:30b",),
    ),
    SoakCase(
        name="reasoning_complex",
        expected_tier="complex",
        prompt=(
            "Resuelve paso a paso un plan de capacidad para 2000 req/min con "
            "trade-off de latencia y costo, y justifica supuestos."
        ),
        allowed_models=("qwen3-coder:30b",),
    ),
    SoakCase(
        name="code_complex",
        expected_tier="complex",
        prompt=(
            "Propone arquitectura escalable de microservicios para router IA en "
            "monorepo, con pipeline CI, estrategia de refactor y pruebas."
        ),
        allowed_models=("qwen3-coder:30b", "deepseek-coder-v2:16b"),
    ),
]


def _post_multi_ai(base_url: str, message: str, timeout: int) -> requests.Response:
    return requests.post(
        f"{base_url.rstrip('/')}/api/brain/chat-multi-ai",
        json={"message": message, "include_routing_info": True},
        timeout=timeout,
    )


def warmup_ollama_models(models: Sequence[str], timeout: int = 120) -> None:
    """
    Calienta modelos en Ollama para evitar timeout de cold-start en pruebas largas.
    """
    ollama_url = "http://127.0.0.1:11434/api/generate"
    for model_name in sorted(set(models)):
        try:
            payload = {
                "model": model_name,
                "prompt": "warmup",
                "stream": False,
                "options": {"num_predict": 8, "temperature": 0.0},
            }
            t0 = time.perf_counter()
            resp = requests.post(ollama_url, json=payload, timeout=timeout)
            dt = round(time.perf_counter() - t0, 2)
            if resp.status_code == 200:
                print(f"[warmup] ok model={model_name} t={dt}s")
            else:
                print(f"[warmup] warn model={model_name} status={resp.status_code}")
        except Exception as ex:
            print(f"[warmup] warn model={model_name} error={ex}")


def run_soak(
    base_url: str, rounds: int, timeout: int, sleep_seconds: float
) -> Dict[str, object]:
    records: List[SoakRecord] = []
    start_ts = time.time()

    for round_idx in range(1, rounds + 1):
        for case in TEST_CASES:
            t0 = time.perf_counter()
            status_code = 0
            model = ""
            provider = ""
            routing_mode = ""
            selected_tier = ""
            complexity_level = ""
            complexity_score = 0.0
            err = ""
            ok = False

            try:
                resp = _post_multi_ai(base_url, case.prompt, timeout=timeout)
                status_code = resp.status_code
                data = resp.json() if resp.content else {}
                model = str(data.get("model") or "")
                provider = str(data.get("provider") or "")
                response_text = str(data.get("response") or "").strip()
                metadata = data.get("metadata") or {}
                routing_info = data.get("routing_info") or {}
                classification = routing_info.get("classification") or {}

                routing_mode = str(metadata.get("routing_mode") or "")
                selected_tier = str(metadata.get("selected_tier") or "")
                complexity_level = str(
                    metadata.get("complexity_level")
                    or classification.get("complexity_level")
                    or ""
                )
                complexity_score = float(
                    metadata.get("complexity_score")
                    or classification.get("complexity_score")
                    or 0.0
                )

                basic_ok = status_code == 200 and bool(response_text) and bool(model)
                tier_ok = model in case.allowed_models
                ok = basic_ok and tier_ok

                if not basic_ok:
                    err = (
                        f"basic_failed(status={status_code}, model={model}, "
                        f"response_len={len(response_text)})"
                    )
                elif not tier_ok:
                    err = (
                        f"unexpected_model(expected={list(case.allowed_models)}, "
                        f"got={model})"
                    )
            except Exception as ex:
                err = str(ex)
                ok = False

            elapsed = round(time.perf_counter() - t0, 2)
            records.append(
                SoakRecord(
                    case_name=case.name,
                    round_idx=round_idx,
                    ok=ok,
                    http_status=status_code,
                    model=model,
                    provider=provider,
                    response_time=elapsed,
                    routing_mode=routing_mode,
                    selected_tier=selected_tier,
                    complexity_level=complexity_level,
                    complexity_score=complexity_score,
                    error=err,
                )
            )
            print(
                f"[round {round_idx:02d}] {case.name:<17} ok={ok:<5} "
                f"model={model or '-':<22} t={elapsed:>5.2f}s err={err or '-'}"
            )
            if sleep_seconds > 0:
                time.sleep(sleep_seconds)

    duration = round(time.time() - start_ts, 2)
    total = len(records)
    passed = sum(1 for r in records if r.ok)
    failed = total - passed

    by_model: Dict[str, int] = {}
    for r in records:
        if r.model:
            by_model[r.model] = by_model.get(r.model, 0) + 1

    by_case: Dict[str, Dict[str, int]] = {}
    for case in TEST_CASES:
        c_recs = [r for r in records if r.case_name == case.name]
        c_ok = sum(1 for r in c_recs if r.ok)
        by_case[case.name] = {"ok": c_ok, "failed": len(c_recs) - c_ok}

    return {
        "timestamp": datetime.now().isoformat(),
        "base_url": base_url,
        "rounds": rounds,
        "duration_seconds": duration,
        "total_requests": total,
        "passed": passed,
        "failed": failed,
        "pass_rate": round((passed / total) * 100, 2) if total else 0.0,
        "by_model": by_model,
        "by_case": by_case,
        "records": [asdict(r) for r in records],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS Multi-AI soak test")
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--timeout", type=int, default=90)
    parser.add_argument("--sleep-seconds", type=float, default=0.25)
    parser.add_argument(
        "--no-warmup",
        action="store_true",
        help="No ejecutar warmup previo de modelos en Ollama.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Ruta opcional para reporte JSON. Default: logs/ai_router_soak_<ts>.json",
    )
    args = parser.parse_args()

    if not args.no_warmup:
        warmup_models = []
        for case in TEST_CASES:
            warmup_models.extend(case.allowed_models)
        warmup_ollama_models(warmup_models)

    report = run_soak(
        base_url=args.base_url,
        rounds=max(1, args.rounds),
        timeout=max(10, args.timeout),
        sleep_seconds=max(0.0, args.sleep_seconds),
    )

    logs_dir = REPO_ROOT / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    if args.output.strip():
        out_path = Path(args.output).resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = logs_dir / f"ai_router_soak_{stamp}.json"

    out_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print("\n=== SOAK SUMMARY ===")
    print(f"duration_s   : {report['duration_seconds']}")
    print(f"total        : {report['total_requests']}")
    print(f"passed       : {report['passed']}")
    print(f"failed       : {report['failed']}")
    print(f"pass_rate    : {report['pass_rate']}%")
    print(f"report       : {out_path}")
    print(f"by_model     : {report['by_model']}")
    print(f"by_case      : {report['by_case']}")

    return 0 if int(report["failed"]) == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
