"""Modelos y límites operativos para autoprogramación gobernada."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional

AUTONOMY_OFF = "off"
AUTONOMY_SAFE = "safe"
AUTONOMY_BUILD = "build"
VALID_AUTONOMY_MODES = {AUTONOMY_OFF, AUTONOMY_SAFE, AUTONOMY_BUILD}
DEFAULT_AUTONOMY_MODE = AUTONOMY_SAFE

MAX_PATCHES_PER_CYCLE = 2
AUTO_COMMIT = False
AUTO_PUSH = False
ALLOW_NEW_FILES = True
ALLOW_DELETE_FILES = False
REQUIRE_COMPILE_CHECK = True
REQUIRE_TESTS_IF_PRESENT = True

ALLOWED_WRITE_ROOTS = ("modules", "atlas_adapter", "bridge", "tests")
PROTECTED_PATH_SEGMENTS = (".git", ".codex", "secrets", "config")


@dataclass
class PolicyDecision:
    allowed: bool
    reason: str
    risk: str
    normalized_target: str = ""
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class PatchBlock:
    old_text: str
    new_text: str
    replace_all: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ChangeProposal:
    topic: str
    source: str
    action: str
    target_file: str
    patches: List[PatchBlock] = field(default_factory=list)
    allow_new_file: bool = False
    metadata: Dict[str, Any] = field(default_factory=dict)
    test_paths: List[str] = field(default_factory=list)
    proposal_id: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class PatchExecutionResult:
    ok: bool
    changed: bool
    target_file: str
    changed_files: List[str] = field(default_factory=list)
    error: Optional[str] = None
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ValidationResult:
    ok: bool
    compile_ok: bool
    tests_ok: Optional[bool]
    ran_tests: bool
    compile_output: str = ""
    tests_output: str = ""
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class AutoProgrammerResult:
    ok: bool
    topic: str
    source: str
    target_file: str
    action: str
    rollback_applied: bool
    validation_result: Dict[str, Any]
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

