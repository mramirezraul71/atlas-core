"""
Generador de código para nuevas skills (stub con validación básica).
"""
from __future__ import annotations

import ast
from typing import Any, Dict, List

# Palabras prohibidas por seguridad
FORBIDDEN = {"os.system", "subprocess", "eval(", "exec(", "__import__", "open(", "file(", "compile("}


class CodeGenerator:
    """Genera y valida código de skills."""

    def validate_code(self, code: str) -> Dict[str, Any]:
        """
        Valida sintaxis y seguridad del código.
        Returns: { syntax_valid, safe, issues[] }
        """
        issues: List[str] = []
        try:
            ast.parse(code)
            syntax_valid = True
        except SyntaxError as e:
            syntax_valid = False
            issues.append(f"Syntax error: {e}")
            return {"syntax_valid": False, "safe": False, "issues": issues}

        for pattern in FORBIDDEN:
            if pattern in code:
                issues.append(f"Forbidden pattern: {pattern}")
        safe = len(issues) == 0
        return {"syntax_valid": True, "safe": safe, "issues": issues}
