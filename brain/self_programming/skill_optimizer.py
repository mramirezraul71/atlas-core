"""
Optimización automática de código generado (Fase 4.3).
"""
from __future__ import annotations

import re
from typing import Dict, List


class SkillOptimizer:
    """Optimiza código generado: manejo de errores, type hints, docstrings, imports."""

    def optimize_code(self, code: str) -> Dict:
        """Aplica optimizaciones y retorna original_code, optimized_code, optimizations, improvement_score."""
        optimizations_applied: List[str] = []
        optimized_code = code
        if "try:" not in optimized_code and "except" not in optimized_code:
            optimized_code = self._add_error_handling(optimized_code)
            optimizations_applied.append("error_handling")
        if "->" not in optimized_code and "def " in optimized_code:
            optimized_code = self._add_type_hints(optimized_code)
            optimizations_applied.append("type_hints")
        optimized_code = self._improve_docstring(optimized_code)
        optimizations_applied.append("docstring")
        cleaned = self._remove_unused_imports(optimized_code)
        if cleaned != optimized_code:
            optimized_code = cleaned
            optimizations_applied.append("unused_imports")
        return {
            "original_code": code,
            "optimized_code": optimized_code,
            "optimizations": optimizations_applied,
            "improvement_score": len(optimizations_applied) / 4.0,
        }

    def _add_error_handling(self, code: str) -> str:
        lines = code.split("\n")
        func_start = -1
        for i, line in enumerate(lines):
            if line.strip().startswith("def "):
                func_start = i
                break
        if func_start == -1:
            return code
        indent = len(lines[func_start]) - len(lines[func_start].lstrip())
        body_indent = indent + 4
        new_lines = lines[: func_start + 1]
        new_lines.append(" " * body_indent + "try:")
        for line in lines[func_start + 1 :]:
            if line.strip():
                new_lines.append(" " * 4 + line)
            else:
                new_lines.append(line)
        new_lines.append(" " * body_indent + "except Exception as e:")
        new_lines.append(" " * (body_indent + 4) + "return {'success': False, 'error': str(e)}")
        return "\n".join(new_lines)

    def _add_type_hints(self, code: str) -> str:
        code = re.sub(r"(def \w+\([^)]*\)):", r"\1 -> Dict:", code)
        if "-> Dict" in code and "from typing import" not in code and "import Dict" not in code:
            code = "from typing import Dict, Optional, List\n\n" + code
        return code

    def _improve_docstring(self, code: str) -> str:
        lines = code.split("\n")
        func_line = -1
        for i, line in enumerate(lines):
            if line.strip().startswith("def "):
                func_line = i
                break
        if func_line == -1:
            return code
        has_docstring = len(lines) > func_line + 1 and '"""' in lines[func_line + 1]
        if not has_docstring:
            indent = len(lines[func_line]) - len(lines[func_line].lstrip())
            docstring = " " * (indent + 4) + '"""Generated skill - auto-optimized"""'
            lines.insert(func_line + 1, docstring)
        return "\n".join(lines)

    def _remove_unused_imports(self, code: str) -> str:
        lines = code.split("\n")
        import_lines = []
        code_lines = []
        for line in lines:
            if line.strip().startswith("import ") or line.strip().startswith("from "):
                import_lines.append(line)
            else:
                code_lines.append(line)
        code_body = "\n".join(code_lines)
        used_imports = []
        for imp_line in import_lines:
            if "import " in imp_line:
                module = imp_line.split("import ")[-1].split(" as ")[0].split(",")[0].strip()
                if module in code_body:
                    used_imports.append(imp_line)
        return "\n".join(used_imports + [""] + code_lines)
