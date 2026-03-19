"""Natural-language parsing and safety validation for ATLAS OpenClaw commands."""
from __future__ import annotations

import json
import re
import unicodedata
import urllib.error
import urllib.request
from typing import Any, Dict, Optional

from .models import ParsedCommand


JOINT_LIMIT_ALIASES = {
    "r_shoulder_pitch": "right_shoulder_pitch",
    "l_shoulder_pitch": "left_shoulder_pitch",
    "r_elbow": "right_elbow",
    "l_elbow": "left_elbow",
}


class NaturalLanguageParser:
    """Conservative natural-language parser for Spanish robot commands."""

    def __init__(self, config: Dict[str, Any]):
        self.config = config

    @staticmethod
    def _normalize(text: str) -> str:
        lowered = text.strip().lower()
        return "".join(
            c for c in unicodedata.normalize("NFD", lowered) if unicodedata.category(c) != "Mn"
        )

    @staticmethod
    def _extract_angle_deg(text: str) -> Optional[float]:
        match = re.search(r"(-?\d{1,3}(?:[\.,]\d+)?)\s*(?:grados|grado|°)?", text)
        if not match:
            return None
        return float(match.group(1).replace(",", "."))

    def parse(self, text: str) -> ParsedCommand:
        normalized = self._normalize(text)

        if any(
            keyword in normalized
            for keyword in ("parada de emergencia", "emergencia", "emergency stop", "corta motor")
        ):
            return ParsedCommand(action="emergency_stop", confidence=1.0, raw_text=text)

        if any(keyword in normalized for keyword in ("deten", "detener", "stop", "parar")):
            return ParsedCommand(action="stop_motion", confidence=0.95, raw_text=text)

        if any(keyword in normalized for keyword in ("home", "posicion inicial", "reposo", "calibracion")):
            return ParsedCommand(action="home_pose", confidence=0.9, raw_text=text)

        if "abr" in normalized and "pinza" in normalized:
            side = "right" if "derech" in normalized else "left" if "izquierd" in normalized else "both"
            return ParsedCommand(action="open_gripper", target=side, confidence=0.9, raw_text=text)

        if ("cerr" in normalized or "cierra" in normalized) and "pinza" in normalized:
            side = "right" if "derech" in normalized else "left" if "izquierd" in normalized else "both"
            return ParsedCommand(action="close_gripper", target=side, confidence=0.9, raw_text=text)

        if "brazo" in normalized or "joint" in normalized or "articulacion" in normalized:
            side = "right" if "derech" in normalized else "left" if "izquierd" in normalized else "right"
            angle = self._extract_angle_deg(normalized)
            if angle is None:
                if any(keyword in normalized for keyword in ("sube", "arriba", "levanta")):
                    angle = 12.0
                elif any(keyword in normalized for keyword in ("baja", "abajo", "desciende")):
                    angle = -12.0
                else:
                    angle = 8.0
            return ParsedCommand(
                action="move_arm",
                target=side,
                params={"angle_deg": angle},
                confidence=0.82,
                raw_text=text,
            )

        llm_result = self._parse_with_ollama(text)
        if llm_result is not None:
            return llm_result

        return ParsedCommand(action="unknown", confidence=0.0, raw_text=text)

    def _parse_with_ollama(self, text: str) -> Optional[ParsedCommand]:
        llm_cfg = self.config.get("llm", {})
        if not llm_cfg.get("enabled", True):
            return None
        if llm_cfg.get("provider", "ollama") != "ollama":
            return None

        base_url = str(llm_cfg.get("base_url", "http://127.0.0.1:11434")).rstrip("/")
        fallback_urls = [str(url).rstrip("/") for url in llm_cfg.get("fallback_urls", []) if str(url).strip()]
        model = llm_cfg.get("model", "llama3.1:8b-instruct")
        timeout_sec = int(llm_cfg.get("timeout_sec", 8))
        schema_help = (
            '{"action":"move_arm|move_joint|open_gripper|close_gripper|home_pose|stop_motion|emergency_stop|unknown",'
            '"target":"right|left|both|",'
            '"params":{"angle_deg":number},"confidence":0.0}'
        )
        prompt = (
            "Extrae una orden de robot de este texto y responde SOLO JSON valido.\n"
            f"Esquema: {schema_help}\n"
            f"Texto: {text}"
        )
        payload = {
            "model": model,
            "messages": [
                {"role": "system", "content": "Eres un parser de comandos para robot. Sin explicaciones."},
                {"role": "user", "content": prompt},
            ],
            "stream": False,
        }

        for url in [base_url, *fallback_urls]:
            request = urllib.request.Request(
                f"{url}/api/chat",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=timeout_sec) as response:
                    body = json.loads(response.read().decode("utf-8", errors="replace"))
                content = (body.get("message") or {}).get("content", "").strip()
                json_start = content.find("{")
                json_end = content.rfind("}")
                if json_start == -1 or json_end == -1:
                    continue
                parsed = json.loads(content[json_start : json_end + 1])
                return ParsedCommand(
                    action=str(parsed.get("action", "unknown")),
                    target=str(parsed.get("target", "")),
                    params=dict(parsed.get("params") or {}),
                    source="ollama",
                    confidence=float(parsed.get("confidence", 0.5)),
                    raw_text=text,
                )
            except (urllib.error.URLError, TimeoutError, ValueError, json.JSONDecodeError):
                continue
        return None


class CommandValidator:
    """Validates parsed commands against explicit safety constraints."""

    def __init__(self, config: Dict[str, Any]):
        safety = config.get("safety", {})
        self.allowed_actions = set(safety.get("allowed_actions", []))
        self.joint_limits = dict(safety.get("joint_limits_deg", {}))
        self.max_single_step = float(safety.get("max_single_step_deg", 40))

    def validate(self, parsed: ParsedCommand) -> Dict[str, Any]:
        if parsed.action not in self.allowed_actions:
            return {"allowed": False, "reason": f"action_not_allowed:{parsed.action}", "risk": "high"}

        if parsed.action in {"emergency_stop", "stop_motion", "home_pose", "open_gripper", "close_gripper"}:
            return {"allowed": True, "reason": "ok", "risk": "low"}

        if parsed.action == "move_arm":
            angle = float(parsed.params.get("angle_deg", 0))
            if abs(angle) > self.max_single_step:
                return {
                    "allowed": False,
                    "reason": f"angle_step_exceeds_limit:{angle}",
                    "risk": "high",
                }
            if parsed.target not in {"right", "left"}:
                return {
                    "allowed": False,
                    "reason": f"invalid_arm_target:{parsed.target}",
                    "risk": "medium",
                }
            return {"allowed": True, "reason": "ok", "risk": "medium"}

        if parsed.action == "move_joint":
            joint = str(parsed.params.get("joint", "")).strip()
            angle = float(parsed.params.get("angle_deg", 0))
            lookup_joint = JOINT_LIMIT_ALIASES.get(joint, joint)
            if lookup_joint not in self.joint_limits:
                return {"allowed": False, "reason": f"unknown_joint:{joint}", "risk": "high"}
            low, high = self.joint_limits[lookup_joint]
            if not (low <= angle <= high):
                return {
                    "allowed": False,
                    "reason": f"angle_out_of_joint_range:{lookup_joint}:{angle}",
                    "risk": "high",
                }
            return {"allowed": True, "reason": "ok", "risk": "medium"}

        return {"allowed": True, "reason": "ok", "risk": "medium"}
