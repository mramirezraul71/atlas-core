"""
ATLAS NEXUS - AI Router
Sistema de routing automático para múltiples proveedores de IA
"""

import logging
import os
import time
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import requests
import yaml

from .query_classifier import QueryType, get_query_classifier

logger = logging.getLogger(__name__)


class ProviderStatus(Enum):
    """Estados de los proveedores"""

    AVAILABLE = "available"
    UNAVAILABLE = "unavailable"
    ERROR = "error"


class AIRouter:
    """Router inteligente para múltiples proveedores de IA"""

    def __init__(self, config_path: str = None):
        """
        Inicializa el AI Router

        Args:
            config_path: Ruta al archivo de configuración YAML
        """
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(__file__), "..", "..", "config", "ai_config.yaml"
            )

        self.config_path = config_path
        self.config = self._load_config()
        self.provider_status = {}
        self._last_routing_context: Dict[str, Any] = {}
        self.query_classifier = get_query_classifier()

        # Verificar disponibilidad de proveedores
        self._check_provider_availability()

    def _load_config(self) -> Dict:
        """Carga la configuración desde archivo YAML"""
        try:
            with open(self.config_path, "r", encoding="utf-8") as file:
                config = yaml.safe_load(file)
                logger.info(f"✅ Configuración cargada desde {self.config_path}")
                return config
        except Exception as e:
            logger.error(f"❌ Error cargando configuración: {e}")
            raise

    def _check_provider_availability(self):
        """Verifica la disponibilidad de cada proveedor"""
        for provider_name, provider_config in self.config.get("providers", {}).items():
            status = ProviderStatus.UNAVAILABLE

            if provider_name == "ollama":
                status = self._check_ollama_availability(provider_config)
            elif provider_name == "deepseek":
                status = self._check_deepseek_availability(provider_config)
            elif provider_name == "openai":
                status = self._check_openai_availability(provider_config)

            self.provider_status[provider_name] = status
            logger.info(f"🔍 {provider_name}: {status.value}")

    def _check_ollama_availability(self, config: Dict) -> ProviderStatus:
        """Verifica si Ollama está disponible en localhost:11434"""
        try:
            base_url = config.get("base_url", "http://localhost:11434")
            response = requests.get(f"{base_url}/api/tags", timeout=5)

            if response.status_code == 200:
                models = response.json().get("models", [])
                available_models = [model["name"] for model in models]
                logger.info(f"🤖 Ollama disponible con modelos: {available_models}")
                return ProviderStatus.AVAILABLE
            else:
                return ProviderStatus.UNAVAILABLE

        except Exception as e:
            logger.warning(f"⚠️ Ollama no disponible: {e}")
            return ProviderStatus.UNAVAILABLE

    def _check_deepseek_availability(self, config: Dict) -> ProviderStatus:
        """Verifica si DeepSeek API está disponible"""
        try:
            api_key = os.getenv(config.get("api_key_env", "DEEPSEEK_API_KEY"))
            if not api_key:
                logger.warning("⚠️ DeepSeek API key no configurada")
                return ProviderStatus.UNAVAILABLE

            # Verificar con un request simple
            response = requests.get(
                f"{config.get('base_url')}/models",
                headers={"Authorization": f"Bearer {api_key}"},
                timeout=5,
            )

            if response.status_code == 200:
                logger.info("🧠 DeepSeek API disponible")
                return ProviderStatus.AVAILABLE
            else:
                return ProviderStatus.UNAVAILABLE

        except Exception as e:
            logger.warning(f"⚠️ DeepSeek API no disponible: {e}")
            return ProviderStatus.UNAVAILABLE

    def _check_openai_availability(self, config: Dict) -> ProviderStatus:
        """Verifica si OpenAI API está disponible"""
        try:
            api_key = os.getenv(config.get("api_key_env", "OPENAI_API_KEY"))
            if not api_key:
                logger.warning("⚠️ OpenAI API key no configurada")
                return ProviderStatus.UNAVAILABLE

            # Verificar con un request simple
            response = requests.get(
                f"{config.get('base_url')}/models",
                headers={"Authorization": f"Bearer {api_key}"},
                timeout=5,
            )

            if response.status_code == 200:
                logger.info("🔮 OpenAI API disponible")
                return ProviderStatus.AVAILABLE
            else:
                return ProviderStatus.UNAVAILABLE

        except Exception as e:
            logger.warning(f"⚠️ OpenAI API no disponible: {e}")
            return ProviderStatus.UNAVAILABLE

    def route_query(
        self, query: str, query_type: QueryType = None
    ) -> Tuple[str, str, str]:
        """
        Determina el mejor proveedor y modelo para una query

        Args:
            query: Texto de la query
            query_type: Tipo de query (opcional, se detecta automáticamente)

        Returns:
            Tuple (provider_name, model_name, reason)
        """
        # Clasificar y estimar complejidad para smart routing
        query_analysis = self.query_classifier.analyze_query(query)
        if query_type is None:
            query_type = QueryType(query_analysis["type"])
        complexity_level = query_analysis.get("complexity_level", "medium")
        complexity_score = float(query_analysis.get("complexity_score", 0.5))

        # Obtener routing rules
        routing_rules = self.config.get("routing_rules", {}).get(query_type.value, {})
        preferred_model = routing_rules.get("preferred_model", "llama3.1:8b")
        fallback_provider = routing_rules.get("fallback_provider", "ollama")
        smart_cfg = self.config.get("system", {}).get("smart_routing", {})
        simple_max = float(smart_cfg.get("simple_max_complexity", 0.35))
        complex_min = float(smart_cfg.get("complex_min_complexity", 0.68))
        if complexity_score <= simple_max:
            complexity_level = "simple"
        elif complexity_score >= complex_min:
            complexity_level = "complex"
        else:
            complexity_level = "medium"

        # Smart routing: fast model para tareas simples y modelos pesados para complejas
        if bool(smart_cfg.get("enabled", True)):
            tier_model_candidates = self._get_tier_model_candidates(
                routing_rules=routing_rules,
                complexity_level=complexity_level,
                fast_default_model=smart_cfg.get("fast_default_model", "llama3.1:8b"),
            )
            for model_name in tier_model_candidates:
                provider_name = self._find_provider_for_model(model_name)
                if provider_name:
                    reason = (
                        f"Smart routing {complexity_level} ({query_type.value}) -> "
                        f"{provider_name}:{model_name}"
                    )
                    self._last_routing_context = {
                        "query_type": query_type.value,
                        "complexity_level": complexity_level,
                        "complexity_score": complexity_score,
                        "selected_tier": complexity_level,
                        "routing_mode": "smart",
                        "candidate_models": tier_model_candidates,
                    }
                    return provider_name, model_name, reason

        # Intentar proveedores en orden de prioridad
        for provider_name in self._get_providers_by_priority():
            if self.provider_status.get(provider_name) != ProviderStatus.AVAILABLE:
                continue

            provider_config = self.config.get("providers", {}).get(provider_name, {})
            available_models = provider_config.get("models", {})

            # Buscar modelo preferido
            if preferred_model in available_models:
                self._last_routing_context = {
                    "query_type": query_type.value,
                    "complexity_level": complexity_level,
                    "complexity_score": complexity_score,
                    "selected_tier": "normal",
                    "routing_mode": "legacy-preferred",
                }
                return (
                    provider_name,
                    preferred_model,
                    f"Modelo preferido para {query_type.value}",
                )

            # Si no está el preferido, usar cualquier modelo disponible del proveedor
            if available_models:
                model_name = list(available_models.keys())[0]
                self._last_routing_context = {
                    "query_type": query_type.value,
                    "complexity_level": complexity_level,
                    "complexity_score": complexity_score,
                    "selected_tier": "fallback",
                    "routing_mode": "legacy-any-model",
                }
                return (
                    provider_name,
                    model_name,
                    f"Modelo disponible para {query_type.value}",
                )

        # Fallback al proveedor de fallback
        if self.provider_status.get(fallback_provider) == ProviderStatus.AVAILABLE:
            fallback_config = self.config.get("providers", {}).get(
                fallback_provider, {}
            )
            fallback_models = fallback_config.get("models", {})
            if fallback_models:
                model_name = list(fallback_models.keys())[0]
                self._last_routing_context = {
                    "query_type": query_type.value,
                    "complexity_level": complexity_level,
                    "complexity_score": complexity_score,
                    "selected_tier": "fallback",
                    "routing_mode": "fallback-provider",
                }
                return (
                    fallback_provider,
                    model_name,
                    f"Fallback para {query_type.value}",
                )

        # Último recurso: OpenAI si está disponible
        if self.provider_status.get("openai") == ProviderStatus.AVAILABLE:
            openai_config = self.config.get("providers", {}).get("openai", {})
            openai_models = openai_config.get("models", {})
            if openai_models:
                model_name = list(openai_models.keys())[0]
                self._last_routing_context = {
                    "query_type": query_type.value,
                    "complexity_level": complexity_level,
                    "complexity_score": complexity_score,
                    "selected_tier": "last-resort",
                    "routing_mode": "openai-last-resort",
                }
                return "openai", model_name, "Último recurso disponible"

        raise Exception("No hay proveedores de IA disponibles")

    def _get_providers_by_priority(self) -> List[str]:
        """Retorna proveedores ordenados por prioridad"""
        providers = self.config.get("providers", {})
        sorted_providers = sorted(
            providers.items(), key=lambda x: x[1].get("priority", 999)
        )
        return [name for name, _ in sorted_providers]

    def _find_provider_for_model(self, model_name: str) -> Optional[str]:
        """Busca un proveedor disponible que tenga exactamente ese modelo."""
        for provider_name in self._get_providers_by_priority():
            if self.provider_status.get(provider_name) != ProviderStatus.AVAILABLE:
                continue
            models = (
                self.config.get("providers", {}).get(provider_name, {}).get("models", {})
            )
            if model_name in models:
                return provider_name
        return None

    def _get_tier_model_candidates(
        self, routing_rules: Dict[str, Any], complexity_level: str, fast_default_model: str
    ) -> List[str]:
        """Construye la lista de modelos candidatos por complejidad."""
        simple_model = routing_rules.get("simple_model")
        normal_model = routing_rules.get("normal_model") or routing_rules.get(
            "preferred_model"
        )
        complex_model = routing_rules.get("complex_model") or normal_model

        if complexity_level == "simple":
            ordered = [simple_model, fast_default_model, normal_model, complex_model]
        elif complexity_level == "complex":
            ordered = [complex_model, normal_model, simple_model, fast_default_model]
        else:
            ordered = [normal_model, simple_model, complex_model, fast_default_model]

        seen = set()
        unique_ordered: List[str] = []
        for model_name in ordered:
            if not model_name or model_name in seen:
                continue
            unique_ordered.append(model_name)
            seen.add(model_name)
        return unique_ordered

    def _get_smart_routing_config(self) -> Dict[str, Any]:
        """Obtiene bloque de configuracion smart_routing."""
        return self.config.get("system", {}).get("smart_routing", {}) or {}

    def _get_active_routing_profile_name(self) -> str:
        """
        Determina el perfil activo de smart routing.
        Permite override por variable de entorno ATLAS_ROUTING_PROFILE.
        """
        smart_cfg = self._get_smart_routing_config()
        profiles = smart_cfg.get("profiles", {})
        env_profile = os.getenv("ATLAS_ROUTING_PROFILE", "").strip()
        configured_profile = str(smart_cfg.get("active_profile", "")).strip()
        candidate = env_profile or configured_profile

        if isinstance(profiles, dict) and candidate in profiles:
            return candidate
        return configured_profile or "default"

    def _get_smart_tier_values(
        self, key: str, default_values: Dict[str, int]
    ) -> Dict[str, int]:
        """
        Retorna valores por tier fusionando:
        defaults -> valores globales -> perfil activo.
        """
        smart_cfg = self._get_smart_routing_config()
        profile_name = self._get_active_routing_profile_name()
        profiles = smart_cfg.get("profiles", {})
        profile_cfg = (
            profiles.get(profile_name, {})
            if isinstance(profiles, dict)
            and isinstance(profiles.get(profile_name, {}), dict)
            else {}
        )

        merged: Dict[str, Any] = dict(default_values)
        global_values = smart_cfg.get(key, {})
        if isinstance(global_values, dict):
            merged.update(global_values)
        profile_values = profile_cfg.get(key, {})
        if isinstance(profile_values, dict):
            merged.update(profile_values)

        normalized: Dict[str, int] = {}
        for tier, fallback in default_values.items():
            try:
                normalized[tier] = int(merged.get(tier, fallback))
            except Exception:
                normalized[tier] = int(fallback)
        return normalized

    def _get_effective_max_tokens(self, model_config: Dict[str, Any]) -> int:
        """
        Ajusta max_tokens por tier para mantener respuestas agiles en fast-path
        y permitir mas salida en tareas complejas.
        """
        configured_max = int(model_config.get("max_tokens", 4096))
        tier = str(self._last_routing_context.get("selected_tier") or "medium")
        default_limits = {"simple": 160, "medium": 420, "complex": 900}
        tier_limits = self._get_smart_tier_values("output_tokens", default_limits)
        tier_limit = int(tier_limits.get(tier, default_limits.get(tier, 420)))
        return max(64, min(configured_max, tier_limit))

    def _get_effective_timeout_seconds(self) -> int:
        """Timeout por tier para evitar cortes prematuros en modelos pesados."""
        tier = str(self._last_routing_context.get("selected_tier") or "medium")
        default_timeout = {"simple": 40, "medium": 90, "complex": 150}
        tier_timeout = self._get_smart_tier_values(
            "request_timeout_seconds", default_timeout
        )
        timeout_s = int(tier_timeout.get(tier, default_timeout.get(tier, 90)))
        return max(20, timeout_s)

    def generate_response(self, query: str, query_type: QueryType = None) -> Dict:
        """
        Genera una respuesta usando el mejor proveedor disponible

        Args:
            query: Texto de la query
            query_type: Tipo de query (opcional)

        Returns:
            Diccionario con respuesta y metadata
        """
        start_time = time.time()

        try:
            # Determinar proveedor y modelo
            provider_name, model_name, reason = self.route_query(query, query_type)

            # Generar respuesta según el proveedor
            if provider_name == "ollama":
                response = self._generate_ollama(query, model_name)
            elif provider_name == "deepseek":
                response = self._generate_deepseek(query, model_name)
            elif provider_name == "openai":
                response = self._generate_openai(query, model_name)
            else:
                raise Exception(f"Proveedor no soportado: {provider_name}")

            # Calcular metadata
            end_time = time.time()
            response_time = end_time - start_time

            provider_config = self.config.get("providers", {}).get(provider_name, {})
            cost = provider_config.get("cost", "unknown")

            return {
                "success": True,
                "response": response,
                "provider": provider_name,
                "model": model_name,
                "reason": reason,
                "cost": cost,
                "response_time": round(response_time, 2),
                "query_type": self._last_routing_context.get(
                    "query_type", query_type.value if query_type else "unknown"
                ),
                "metadata": {
                    "provider_priority": provider_config.get("priority", 999),
                    "model_specialty": provider_config.get("models", {})
                    .get(model_name, {})
                    .get("specialty", "general"),
                    "max_tokens": provider_config.get("models", {})
                    .get(model_name, {})
                    .get("max_tokens", 4096),
                    "routing_mode": self._last_routing_context.get("routing_mode"),
                    "selected_tier": self._last_routing_context.get("selected_tier"),
                    "routing_profile": self._get_active_routing_profile_name(),
                    "complexity_level": self._last_routing_context.get(
                        "complexity_level"
                    ),
                    "complexity_score": self._last_routing_context.get(
                        "complexity_score"
                    ),
                    "candidate_models": self._last_routing_context.get(
                        "candidate_models", []
                    ),
                },
            }

        except Exception as e:
            logger.error(f"❌ Error generando respuesta: {e}")
            return {
                "success": False,
                "error": str(e),
                "response": "Lo siento, no puedo procesar tu solicitud en este momento. Por favor, intenta más tarde.",
                "provider": "none",
                "model": "none",
                "reason": "Error en generación",
                "cost": "unknown",
                "response_time": 0,
                "query_type": query_type.value if query_type else "unknown",
            }

    def _generate_ollama(self, query: str, model: str) -> str:
        """Genera respuesta usando Ollama"""
        try:
            provider_config = self.config.get("providers", {}).get("ollama", {})
            base_url = provider_config.get("base_url", "http://localhost:11434")
            model_config = provider_config.get("models", {}).get(model, {})

            payload = {
                "model": model,
                "prompt": query,
                "stream": False,
                "options": {
                    "temperature": model_config.get("temperature", 0.7),
                    "num_predict": self._get_effective_max_tokens(model_config),
                },
            }

            response = requests.post(
                f"{base_url}/api/generate",
                json=payload,
                timeout=self._get_effective_timeout_seconds(),
            )

            if response.status_code == 200:
                result = response.json()
                return result.get("response", "Respuesta vacía")
            else:
                raise Exception(f"Ollama error: {response.status_code}")

        except Exception as e:
            logger.error(f"❌ Error con Ollama: {e}")
            raise

    def _generate_deepseek(self, query: str, model: str) -> str:
        """Genera respuesta usando DeepSeek API"""
        try:
            provider_config = self.config.get("providers", {}).get("deepseek", {})
            api_key = os.getenv(provider_config.get("api_key_env", "DEEPSEEK_API_KEY"))
            model_config = provider_config.get("models", {}).get(model, {})

            if not api_key:
                raise Exception("DeepSeek API key no configurada")

            payload = {
                "model": model,
                "messages": [{"role": "user", "content": query}],
                "max_tokens": self._get_effective_max_tokens(model_config),
                "temperature": model_config.get("temperature", 0.7),
                "stream": False,
            }

            headers = {
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            }

            response = requests.post(
                f"{provider_config.get('base_url')}/chat/completions",
                json=payload,
                headers=headers,
                timeout=self._get_effective_timeout_seconds(),
            )

            if response.status_code == 200:
                result = response.json()
                return result["choices"][0]["message"]["content"]
            else:
                raise Exception(f"DeepSeek error: {response.status_code}")

        except Exception as e:
            logger.error(f"❌ Error con DeepSeek: {e}")
            raise

    def _generate_openai(self, query: str, model: str) -> str:
        """Genera respuesta usando OpenAI API"""
        try:
            provider_config = self.config.get("providers", {}).get("openai", {})
            api_key = os.getenv(provider_config.get("api_key_env", "OPENAI_API_KEY"))
            model_config = provider_config.get("models", {}).get(model, {})

            if not api_key:
                raise Exception("OpenAI API key no configurada")

            payload = {
                "model": model,
                "messages": [{"role": "user", "content": query}],
                "max_tokens": self._get_effective_max_tokens(model_config),
                "temperature": model_config.get("temperature", 0.7),
                "stream": False,
            }

            headers = {
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            }

            response = requests.post(
                f"{provider_config.get('base_url')}/chat/completions",
                json=payload,
                headers=headers,
                timeout=self._get_effective_timeout_seconds(),
            )

            if response.status_code == 200:
                result = response.json()
                return result["choices"][0]["message"]["content"]
            else:
                raise Exception(f"OpenAI error: {response.status_code}")

        except Exception as e:
            logger.error(f"❌ Error con OpenAI: {e}")
            raise

    def get_available_providers(self) -> Dict:
        """Retorna el estado de todos los proveedores"""
        providers_info = {}

        for provider_name, provider_config in self.config.get("providers", {}).items():
            status = self.provider_status.get(provider_name, ProviderStatus.UNAVAILABLE)

            providers_info[provider_name] = {
                "name": provider_config.get("name", provider_name),
                "status": status.value,
                "priority": provider_config.get("priority", 999),
                "cost": provider_config.get("cost", "unknown"),
                "available_models": list(provider_config.get("models", {}).keys())
                if status == ProviderStatus.AVAILABLE
                else [],
                "base_url": provider_config.get("base_url", ""),
            }

        return providers_info

    def get_routing_info(self, query: str) -> Dict:
        """Retorna información de routing para una query específica"""
        classification = self.query_classifier.get_classification_details(query)
        query_type = QueryType(classification["type"])

        try:
            provider_name, model_name, reason = self.route_query(query, query_type)

            return {
                "classification": classification,
                "routing": {
                    "selected_provider": provider_name,
                    "selected_model": model_name,
                    "reason": reason,
                    "provider_status": self.provider_status.get(
                        provider_name, ProviderStatus.UNAVAILABLE
                    ).value,
                    "provider_cost": self.config.get("providers", {})
                    .get(provider_name, {})
                    .get("cost", "unknown"),
                },
            }
        except Exception as e:
            return {
                "classification": classification,
                "routing": {
                    "error": str(e),
                    "selected_provider": "none",
                    "selected_model": "none",
                    "reason": "No hay proveedores disponibles",
                },
            }


# Singleton instance
_ai_router = None


def get_ai_router() -> AIRouter:
    """Get global AI router instance"""
    global _ai_router
    if _ai_router is None:
        _ai_router = AIRouter()
    return _ai_router
