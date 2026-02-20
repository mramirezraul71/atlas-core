"""Consulta al LLM cuando el robot tiene dudas (aprendizaje progresivo ATLAS). Usa router ATLAS."""
from __future__ import annotations

import asyncio
import json
import os
import re
import time
from datetime import datetime
from typing import Any, Dict, List, Optional


class AIConsultant:
    """
    Interfaz para consultar el LLM cuando el robot está incierto.
    Construcción de prompts contextuales, extracción de conocimiento, validación.
    Usa modules.humanoid.ai.router (Ollama free-first).
    """

    def __init__(
        self,
        llm_endpoint: Optional[str] = None,
        model: Optional[str] = None,
        fallback_model: Optional[str] = None,
    ) -> None:
        self.llm_endpoint = llm_endpoint
        self.bedrock_tutor_model = os.getenv(
            "ATLAS_BEDROCK_TUTOR_MODEL", "us.anthropic.claude-opus-4-6-v1:0"
        )
        self.bedrock_fast_model = os.getenv(
            "ATLAS_BEDROCK_FAST_MODEL", "us.anthropic.claude-haiku-4-5-20251001-v1:0"
        )
        self.direct_tutor_model = os.getenv(
            "ATLAS_DIRECT_TUTOR_MODEL", "claude-opus-4-1-20250805"
        )
        self.direct_fast_model = os.getenv(
            "ATLAS_DIRECT_FAST_MODEL", "claude-3-5-haiku-20241022"
        )
        self.model = model or self.bedrock_tutor_model
        self.fallback_model = fallback_model or self.bedrock_fast_model
        self.ai_mode = (os.getenv("ATLAS_AI_MODE", "bedrock") or "bedrock").strip().lower()
        self.aws_region = (os.getenv("AWS_REGION", "us-east-1") or "us-east-1").strip()
        self.stats: Dict[str, Any] = {
            "total_consultations": 0,
            "successful_consultations": 0,
            "failed_consultations": 0,
            "total_knowledge_extracted": 0,
            "consultation_times": [],
        }

    def ask_for_guidance(
        self,
        situation: str,
        context: Dict[str, Any],
        uncertainty_reason: str,
        previous_attempts: Optional[List[Dict[str, Any]]] = None,
        consultation_type: str = "general",
    ) -> Dict[str, Any]:
        """
        Consultar LLM para obtener guía.
        Returns: guidance, reasoning, suggested_action, new_knowledge, confidence, consultation_successful
        """
        start_time = time.time()
        self.stats["total_consultations"] += 1

        prompt = self._build_consultation_prompt(
            situation, context, uncertainty_reason, previous_attempts or [], consultation_type
        )
        response = self._call_llm(prompt, model=self.model)
        if not response or "Error" in response:
            response = self._call_llm(prompt, model=self.fallback_model)

        if response and "Error" not in response:
            self.stats["successful_consultations"] += 1
            knowledge = self._extract_knowledge(response)
            self.stats["total_knowledge_extracted"] += len(knowledge)
            result = {
                "guidance": response,
                "reasoning": self._extract_reasoning(response),
                "suggested_action": self._extract_action(response),
                "new_knowledge": knowledge,
                "confidence": self._extract_confidence(response),
                "consultation_successful": True,
                "consultation_type": consultation_type,
                "model_used": self.model,
            }
        else:
            self.stats["failed_consultations"] += 1
            result = {
                "guidance": "LLM consultation failed",
                "reasoning": "",
                "suggested_action": "retry_or_ask_human",
                "new_knowledge": [],
                "confidence": 0.0,
                "consultation_successful": False,
                "error": response,
            }

        self.stats["consultation_times"].append(time.time() - start_time)
        return result

    def _build_consultation_prompt(
        self,
        situation: str,
        context: Dict[str, Any],
        uncertainty_reason: str,
        previous_attempts: List[Dict[str, Any]],
        consultation_type: str,
    ) -> str:
        """Construir prompt estructurado según tipo de consulta."""
        prompt = f"""Soy ATLAS, un robot autónomo con capacidad de aprendizaje continuo.
Necesito tu ayuda para resolver una situación en la que estoy incierto.

TIPO DE CONSULTA: {consultation_type}

"""

        if consultation_type == "identification":
            prompt += f"""OBJETO/SITUACIÓN A IDENTIFICAR:
{situation}

INFORMACIÓN VISUAL DISPONIBLE:
{json.dumps(context.get('visual_info', {}), indent=2, ensure_ascii=False)}

"""
        elif consultation_type == "planning":
            prompt += f"""OBJETIVO:
{situation}

ESTADO ACTUAL:
{json.dumps(context.get('current_state', {}), indent=2, ensure_ascii=False)}

ACCIONES DISPONIBLES:
{json.dumps(context.get('available_actions', []), indent=2, ensure_ascii=False)}

"""
        elif consultation_type == "troubleshooting":
            prompt += f"""PROBLEMA:
{situation}

SÍNTOMAS:
{json.dumps(context.get('symptoms', {}), indent=2, ensure_ascii=False)}

"""
        else:
            prompt += f"""SITUACIÓN:
{situation}

CONTEXTO:
{json.dumps(context, indent=2, ensure_ascii=False)}

"""

        prompt += f"""POR QUÉ ESTOY INCIERTO:
{uncertainty_reason}

"""

        if previous_attempts:
            prompt += "INTENTOS PREVIOS QUE FALLARON:\n"
            for i, attempt in enumerate(previous_attempts, 1):
                action = attempt.get("action", "unknown")
                result_attempt = attempt.get("result", "unknown")
                reason = attempt.get("failure_reason", "unknown")
                prompt += f"{i}. Acción: {action}\n   Resultado: {result_attempt}\n   Razón de fallo: {reason}\n\n"

        prompt += """NECESITO QUE ME PROPORCIONES:

1. Tu razonamiento paso a paso sobre esta situación
2. Qué acción específica debería tomar ahora
3. Conocimiento general que debería aprender para el futuro
4. Tu nivel de confianza en esta recomendación

FORMATO DE RESPUESTA (importante - usa estas etiquetas):

<reasoning>
[Tu razonamiento detallado paso a paso]
- Analiza la situación
- Considera los intentos previos
- Explica tu lógica
</reasoning>

<action>
[Acción específica y concreta que debo tomar]
Describe exactamente qué hacer, con parámetros si es necesario
</action>

<knowledge>
[Conocimiento general para aprender]
Formato: concepto: explicación detallada
Ejemplo: 
fragile_handling: Los objetos frágiles requieren agarre suave y movimiento lento
object_identification: Objetos cilíndricos rojos en cocinas suelen ser extintores

Incluye múltiples piezas de conocimiento separadas si es relevante
</knowledge>

<confidence>
[Tu nivel de confianza: número de 0 a 100]
</confidence>

Sé específico, práctico y educativo. Este conocimiento se guardará en mi memoria.
"""
        return prompt

    def _run_async(self, coro: Any) -> Any:
        """Ejecutar corrutina en contexto síncrono (compatible con hilo/event-loop)."""
        try:
            return asyncio.run(coro)
        except RuntimeError:
            loop = asyncio.new_event_loop()
            try:
                asyncio.set_event_loop(loop)
                return loop.run_until_complete(coro)
            finally:
                try:
                    loop.close()
                except Exception:
                    pass

    async def _call_bedrock_async(self, prompt: str, model: str) -> str:
        from anthropic import AsyncAnthropicBedrock

        client = AsyncAnthropicBedrock(aws_region=self.aws_region)
        msg = await client.messages.create(
            model=model,
            max_tokens=900,
            temperature=0.2,
            messages=[{"role": "user", "content": prompt}],
        )
        parts = []
        for block in getattr(msg, "content", []) or []:
            txt = getattr(block, "text", None)
            if txt:
                parts.append(txt)
        return "\n".join(parts).strip()

    async def _call_direct_async(self, prompt: str, model: str) -> str:
        from anthropic import AsyncAnthropic

        api_key = os.getenv("ANTHROPIC_API_KEY", "")
        if not api_key:
            raise RuntimeError("ANTHROPIC_API_KEY no configurada")
        client = AsyncAnthropic(api_key=api_key)
        msg = await client.messages.create(
            model=model,
            max_tokens=900,
            temperature=0.2,
            messages=[{"role": "user", "content": prompt}],
        )
        parts = []
        for block in getattr(msg, "content", []) or []:
            txt = getattr(block, "text", None)
            if txt:
                parts.append(txt)
        return "\n".join(parts).strip()

    def _normalize_model_for_backend(self, model: str, backend: str) -> str:
        raw = (model or "").strip()
        if backend == "bedrock":
            if raw.startswith("us.anthropic."):
                return raw
            if "haiku" in raw.lower() or raw == self.fallback_model:
                return self.bedrock_fast_model
            return self.bedrock_tutor_model
        if raw.startswith("claude-"):
            return raw
        if "haiku" in raw.lower() or raw == self.fallback_model:
            return self.direct_fast_model
        return self.direct_tutor_model

    def _call_llm(self, prompt: str, model: str) -> str:
        """Llamar LLM con soporte dual Bedrock/Direct Anthropic y fallback automático."""
        mode = self.ai_mode
        backend_order = ["bedrock", "direct"] if mode == "bedrock" else ["direct", "bedrock"]
        if mode not in ("bedrock", "direct"):
            backend_order = ["bedrock", "direct"]

        errors: List[str] = []
        for backend in backend_order:
            try:
                backend_model = self._normalize_model_for_backend(model, backend)
                if backend == "bedrock":
                    out = self._run_async(self._call_bedrock_async(prompt, backend_model))
                else:
                    out = self._run_async(self._call_direct_async(prompt, backend_model))
                if out and out.strip():
                    return out.strip()
                errors.append(f"{backend}:{backend_model} respuesta vacia")
            except Exception as e:
                errors.append(f"{backend}:{str(e)}")
        return "Error: " + " | ".join(errors)

    def _call_llm_legacy_router(self, prompt: str) -> str:
        """Fallback legado vía router ATLAS (solo contingencia)."""
        try:
            from modules.humanoid.ai.router import route_and_run

            response, _decision, _meta = route_and_run(
                prompt,
                intent_hint="reason",
                prefer_free=True,
                timeout_s=90,
            )
            return (response or "").strip()
        except Exception as e:
            return f"Error: {str(e)}"

    def _extract_reasoning(self, response: str) -> str:
        """Extraer razonamiento de respuesta."""
        match = re.search(r"<reasoning>(.*?)</reasoning>", response, re.DOTALL | re.IGNORECASE)
        if match:
            return match.group(1).strip()
        paragraphs = response.split("\n\n")
        return paragraphs[0] if paragraphs else ""

    def _extract_action(self, response: str) -> str:
        """Extraer acción sugerida."""
        match = re.search(r"<action>(.*?)</action>", response, re.DOTALL | re.IGNORECASE)
        if match:
            return match.group(1).strip()
        for line in response.split("\n"):
            if any(w in line.lower() for w in ("should", "recommend", "suggest", "action")):
                return line.strip()
        return "No clear action suggested"

    def _extract_knowledge(self, response: str) -> List[Dict[str, Any]]:
        """Extraer conocimiento estructurado (concept, rule, skill)."""
        items: List[Dict[str, Any]] = []
        match = re.search(r"<knowledge>(.*?)</knowledge>", response, re.DOTALL | re.IGNORECASE)
        if match:
            knowledge_text = match.group(1).strip()
        else:
            knowledge_text = ""
            for i, line in enumerate(response.split("\n")):
                if any(w in line.lower() for w in ("learn", "knowledge", "remember")):
                    lines = response.split("\n")[i : i + 6]
                    knowledge_text = "\n".join(lines)
                    break
            if not knowledge_text:
                return items

        for line in knowledge_text.split("\n"):
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("//"):
                continue
            if ":" in line:
                parts = line.split(":", 1)
                name = re.sub(r"^[-*•]\s*", "", parts[0].strip())
                definition = parts[1].strip()
                knowledge_type = "concept"
                if any(w in name.lower() for w in ("rule", "when", "if")):
                    knowledge_type = "rule"
                elif any(w in name.lower() for w in ("skill", "how to")):
                    knowledge_type = "skill"
                items.append({
                    "type": knowledge_type,
                    "name": name,
                    "definition": definition,
                    "source": "llm_consultation",
                    "confidence": 0.8,
                    "learned_at": datetime.now().isoformat(),
                })
        return items

    def _extract_confidence(self, response: str) -> float:
        """Extraer nivel de confianza (0-100 -> 0.0-1.0)."""
        match = re.search(r"<confidence>\s*(\d+)\s*</confidence>", response, re.IGNORECASE)
        if match:
            return min(int(match.group(1)) / 100.0, 1.0)
        for line in response.split("\n"):
            if "confidence" in line.lower():
                numbers = re.findall(r"\d+", line)
                if numbers:
                    return min(int(numbers[0]) / 100.0, 1.0)
        return 0.7

    def get_statistics(self) -> Dict[str, Any]:
        """Estadísticas de consultas."""
        total = self.stats["total_consultations"]
        times = self.stats["consultation_times"]
        avg_time = sum(times) / len(times) if times else 0
        successful = self.stats["successful_consultations"]
        return {
            "total_consultations": total,
            "successful": successful,
            "failed": self.stats["failed_consultations"],
            "success_rate": successful / total if total > 0 else 0,
            "total_knowledge_extracted": self.stats["total_knowledge_extracted"],
            "avg_knowledge_per_consultation": (
                self.stats["total_knowledge_extracted"] / successful if successful > 0 else 0
            ),
            "avg_consultation_time_seconds": round(avg_time, 2),
        }
