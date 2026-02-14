"""
Ejecución de código generado en contenedor Docker aislado (Fase 4.3).
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from typing import Any, Dict, List

try:
    import docker
    _DOCKER_AVAILABLE = True
except ImportError:
    docker = None
    _DOCKER_AVAILABLE = False


class SkillSandbox:
    """Ejecuta código en contenedor Docker aislado."""

    def __init__(self) -> None:
        self.docker_client = None
        self.sandbox_image = "python:3.11-slim"
        if _DOCKER_AVAILABLE:
            try:
                self.docker_client = docker.from_env()
                self._ensure_image()
            except Exception:
                self.docker_client = None

    def _ensure_image(self) -> None:
        if not self.docker_client:
            return
        try:
            self.docker_client.images.get(self.sandbox_image)
        except docker.errors.ImageNotFound:
            try:
                self.docker_client.images.pull(self.sandbox_image)
            except Exception:
                pass

    def execute_safely(
        self,
        code: str,
        function_name: str,
        test_input: Dict[str, Any],
        timeout: int = 30,
    ) -> Dict[str, Any]:
        """Ejecuta código en contenedor. Retorna success, output, stdout, stderr, execution_time."""
        if not self.docker_client:
            return {"success": False, "error": "Docker not available"}

        with tempfile.TemporaryDirectory() as tmpdir:
            code_file = os.path.join(tmpdir, "skill.py")
            with open(code_file, "w", encoding="utf-8") as f:
                f.write(code)
            exec_script = f"""
import json
import sys
from skill import {function_name}

try:
    input_data = {json.dumps(test_input)}
    result = {function_name}(**input_data)
    print(json.dumps({{'success': True, 'output': result}}))
except Exception as e:
    print(json.dumps({{'success': False, 'error': str(e)}}), file=sys.stderr)
    sys.exit(1)
"""
            exec_file = os.path.join(tmpdir, "execute.py")
            with open(exec_file, "w", encoding="utf-8") as f:
                f.write(exec_script)

            try:
                start_time = time.time()
                container = self.docker_client.containers.run(
                    self.sandbox_image,
                    command="python execute.py",
                    volumes={tmpdir: {"bind": "/app", "mode": "ro"}},
                    working_dir="/app",
                    network_mode="none",
                    mem_limit="512m",
                    detach=True,
                    remove=False,
                )
                try:
                    exit_code = container.wait(timeout=timeout)
                    status_code = exit_code.get("StatusCode", -1) if isinstance(exit_code, dict) else exit_code
                    stdout = container.logs(stdout=True, stderr=False).decode("utf-8", errors="replace")
                    stderr = container.logs(stdout=False, stderr=True).decode("utf-8", errors="replace")
                finally:
                    try:
                        container.remove(force=True)
                    except Exception:
                        pass
                execution_time = time.time() - start_time
                if status_code == 0 and stdout.strip():
                    try:
                        result = json.loads(stdout.strip().split("\n")[-1])
                        result["execution_time"] = execution_time
                        result["stdout"] = stdout
                        result["stderr"] = stderr
                        return result
                    except json.JSONDecodeError:
                        pass
                return {
                    "success": False,
                    "error": stderr or "Execution failed",
                    "execution_time": execution_time,
                    "stdout": stdout,
                    "stderr": stderr,
                }
            except docker.errors.ContainerError as e:
                return {
                    "success": False,
                    "error": str(e),
                    "stdout": e.stdout.decode("utf-8", errors="replace") if getattr(e, "stdout", None) else "",
                    "stderr": e.stderr.decode("utf-8", errors="replace") if getattr(e, "stderr", None) else "",
                }
            except Exception as e:
                return {"success": False, "error": f"Execution error: {e}"}


class SkillValidator:
    """Valida skills generadas (sintaxis, seguridad, sandbox, rendimiento)."""

    def __init__(self) -> None:
        self.sandbox = SkillSandbox()

    def validate_comprehensive(
        self,
        code: str,
        function_name: str,
        test_cases: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Valida: sintaxis, seguridad, ejecución en sandbox, rendimiento."""
        validation_results: Dict[str, Any] = {"overall_valid": False, "checks": {}}
        from brain.self_programming.code_generator import CodeGenerator
        code_gen = CodeGenerator()
        syntax_result = code_gen.validate_code(code)
        validation_results["checks"]["syntax"] = syntax_result
        if not syntax_result.get("syntax_valid"):
            return validation_results
        if not syntax_result.get("safe", True):
            validation_results["checks"]["security"] = {"passed": False, "issues": syntax_result.get("issues", [])}
            return validation_results
        validation_results["checks"]["security"] = {"passed": True}

        if not test_cases:
            validation_results["checks"]["functional"] = {"passed": True, "results": [], "pass_rate": 1.0}
            validation_results["checks"]["performance"] = {"passed": True, "avg_execution_time": 0, "total_time": 0}
            validation_results["overall_valid"] = True
            return validation_results

        test_results = []
        total_time = 0.0
        for test_case in test_cases:
            inp = test_case.get("input", {})
            expected = test_case.get("expected_output")
            result = self.sandbox.execute_safely(code, function_name, inp, timeout=10)
            passed = result.get("success") and result.get("output") == expected
            test_results.append({
                "input": inp,
                "expected": expected,
                "actual": result.get("output"),
                "passed": passed,
                "execution_time": result.get("execution_time", 0),
            })
            total_time += result.get("execution_time", 0)

        validation_results["checks"]["functional"] = {
            "passed": all(t["passed"] for t in test_results),
            "results": test_results,
            "pass_rate": sum(1 for t in test_results if t["passed"]) / len(test_results) if test_results else 0,
        }
        avg_time = total_time / len(test_cases) if test_cases else 0
        validation_results["checks"]["performance"] = {
            "passed": avg_time < 5.0,
            "avg_execution_time": avg_time,
            "total_time": total_time,
        }
        validation_results["overall_valid"] = all(
            c.get("passed", False) for c in validation_results["checks"].values() if isinstance(c, dict)
        )
        return validation_results
