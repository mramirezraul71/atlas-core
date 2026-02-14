"""
RegressionTester - Suite de tests pre-actualización: API, integración, rendimiento.
Compara con baseline para detectar regresiones.
"""
from __future__ import annotations

import logging
import time
import urllib.request
import urllib.error
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


@dataclass
class TestResult:
    name: str
    passed: bool
    duration_ms: float
    message: str = ""
    details: dict = field(default_factory=dict)


@dataclass
class TestResults:
    passed: int
    failed: int
    warnings: int
    results: list[TestResult]
    total_duration_ms: float

    @property
    def all_passed(self) -> bool:
        return self.failed == 0


class RegressionTester:
    """
    Ejecuta tests de API (endpoints críticos), latencia y opcionalmente integración.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("evolution", {})
        self._test_cfg = self._config.get("regression_testing", {})
        self._timeout = int(self._test_cfg.get("timeout_per_test", 30))
        self._base_urls = {
            "push": "http://127.0.0.1:8791",
            "nexus": "http://127.0.0.1:8000",
            "robot": "http://127.0.0.1:8002",
        }

    def _check_url(self, url: str, timeout: int | None = None) -> tuple[bool, float, str]:
        """GET url; retorna (ok, latency_ms, message)."""
        timeout = timeout or self._timeout
        start = time.perf_counter()
        try:
            req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                latency = (time.perf_counter() - start) * 1000
                return (r.status == 200, latency, f"HTTP {r.status}")
        except urllib.error.HTTPError as e:
            latency = (time.perf_counter() - start) * 1000
            return (False, latency, f"HTTP {e.code}")
        except Exception as e:
            latency = (time.perf_counter() - start) * 1000
            return (False, latency, str(e)[:200])

    def run_api_tests(self) -> list[TestResult]:
        """Comprueba endpoints críticos de PUSH, NEXUS, Robot."""
        results = []
        endpoints = [
            ("push_health", f"{self._base_urls['push']}/health"),
            ("push_status", f"{self._base_urls['push']}/status"),
            ("nexus_health", f"{self._base_urls['nexus']}/health"),
            ("nexus_status", f"{self._base_urls['nexus']}/status"),
            ("robot_root", f"{self._base_urls['robot']}/"),
        ]
        for name, url in endpoints:
            ok, lat_ms, msg = self._check_url(url)
            results.append(TestResult(
                name=name,
                passed=ok,
                duration_ms=round(lat_ms, 2),
                message=msg,
                details={"url": url},
            ))
        return results

    def run_performance_benchmark(self) -> dict[str, float]:
        """Mide latencias de cada servicio; retorna dict name -> latency_ms."""
        out = {}
        for name, base in self._base_urls.items():
            url = f"{base}/health" if name != "robot" else f"{base}/"
            ok, lat_ms, _ = self._check_url(url, timeout=5)
            out[name] = round(lat_ms, 2)
        return out

    def compare_with_baseline(self, current_metrics: dict[str, float], baseline: dict[str, float], max_degradation_pct: float = 10) -> tuple[bool, list[str]]:
        """True si no hay regresión; lista de mensajes de regresión."""
        regressions = []
        for key, current in current_metrics.items():
            base_val = baseline.get(key)
            if base_val is None or base_val == 0:
                continue
            pct = ((current - base_val) / base_val) * 100
            if pct > max_degradation_pct:
                regressions.append(f"{key}: latency +{pct:.1f}% (current={current}ms, baseline={base_val}ms)")
        return (len(regressions) == 0, regressions)

    def run_full_test_suite(self) -> TestResults:
        """Ejecuta API tests y devuelve TestResults."""
        start = time.perf_counter()
        api_results = self.run_api_tests()
        passed = sum(1 for r in api_results if r.passed)
        failed = sum(1 for r in api_results if not r.passed)
        warnings = 0
        total = (time.perf_counter() - start) * 1000
        return TestResults(
            passed=passed,
            failed=failed,
            warnings=warnings,
            results=api_results,
            total_duration_ms=round(total, 2),
        )

    def generate_test_report(self) -> str:
        """Reporte en texto de la última suite."""
        results = self.run_full_test_suite()
        lines = [
            "=== Regression Test Report ===",
            f"Passed: {results.passed}, Failed: {results.failed}, Warnings: {results.warnings}",
            f"Total duration: {results.total_duration_ms}ms",
            "",
        ]
        for r in results.results:
            status = "PASS" if r.passed else "FAIL"
            lines.append(f"  [{status}] {r.name} ({r.duration_ms}ms) {r.message}")
        return "\n".join(lines)
