"""
ATLAS Turbo Configuration
==========================
Configuración optimizada para máxima velocidad y coherencia.

Este módulo define los parámetros de rendimiento del sistema.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Any


@dataclass(frozen=True)
class TurboConfig:
    """
    Configuración de alto rendimiento para ATLAS.
    
    Todos los valores están optimizados para:
    - Mínima latencia de respuesta
    - Máximo throughput de operaciones
    - Coherencia técnica (valores consistentes)
    """
    
    # =========================================================================
    # INTERVALOS DE POLLING (segundos)
    # =========================================================================
    
    # Health checks - cada cuánto verificar salud
    HEALTH_CHECK_INTERVAL: float = 5.0  # Era 30s, ahora 5s
    
    # Trigger engine - cada cuánto evaluar condiciones
    TRIGGER_EVAL_INTERVAL: float = 2.0  # Era 60s, ahora 2s
    
    # Watchdog - cada cuánto verificar componentes
    WATCHDOG_INTERVAL: float = 3.0  # Era 10s, ahora 3s
    
    # Dispatcher queue - polling del queue
    DISPATCHER_POLL_INTERVAL: float = 0.1  # 100ms para respuesta inmediata
    
    # =========================================================================
    # TIMEOUTS (segundos) - COHERENCIA
    # =========================================================================
    
    # HTTP requests
    HTTP_TIMEOUT_FAST: int = 5      # Para health checks
    HTTP_TIMEOUT_NORMAL: int = 15   # Para operaciones normales
    HTTP_TIMEOUT_SLOW: int = 60     # Para operaciones largas
    
    # Shell commands
    SHELL_TIMEOUT_FAST: int = 10    # Comandos simples
    SHELL_TIMEOUT_NORMAL: int = 30  # Comandos normales
    SHELL_TIMEOUT_SLOW: int = 120   # Comandos largos (git pull, etc)
    
    # POT execution
    POT_STEP_TIMEOUT: int = 60      # Timeout por paso
    POT_TOTAL_TIMEOUT: int = 300    # Timeout total del POT
    
    # =========================================================================
    # RETRIES - COHERENCIA
    # =========================================================================
    
    # Número de reintentos por defecto
    DEFAULT_RETRIES: int = 3
    
    # Backoff entre reintentos (segundos)
    RETRY_BACKOFF_BASE: float = 1.0
    RETRY_BACKOFF_MAX: float = 30.0
    RETRY_BACKOFF_MULTIPLIER: float = 2.0
    
    # =========================================================================
    # CONCURRENCIA
    # =========================================================================
    
    # Thread pool para executor
    EXECUTOR_POOL_SIZE: int = 4
    
    # Máximo de POTs ejecutándose simultáneamente
    MAX_CONCURRENT_POTS: int = 3
    
    # Máximo de health checks en paralelo
    MAX_CONCURRENT_HEALTH_CHECKS: int = 8
    
    # =========================================================================
    # QUEUE SIZES
    # =========================================================================
    
    # Tamaño máximo del queue de dispatch
    DISPATCH_QUEUE_SIZE: int = 100
    
    # Historial máximo de ejecuciones
    EXECUTION_HISTORY_SIZE: int = 500
    
    # =========================================================================
    # COOLDOWNS (segundos)
    # =========================================================================
    
    # Mínimo entre triggers del mismo tipo
    TRIGGER_COOLDOWN_MIN: int = 10
    
    # Cooldown para auto-commit
    AUTO_COMMIT_COOLDOWN: int = 60  # Era 600s, ahora 60s
    
    # Cooldown para auto-repair
    AUTO_REPAIR_COOLDOWN: int = 30  # Era 120s, ahora 30s
    
    # =========================================================================
    # LOGGING
    # =========================================================================
    
    # Nivel de logging (DEBUG=10, INFO=20, WARNING=30)
    LOG_LEVEL: int = 20  # INFO por defecto
    
    # Log cada N operaciones (reduce I/O)
    LOG_BATCH_SIZE: int = 10
    
    # =========================================================================
    # MÉTODOS
    # =========================================================================
    
    def to_dict(self) -> Dict[str, Any]:
        """Convierte la config a diccionario."""
        return {
            "health_check_interval": self.HEALTH_CHECK_INTERVAL,
            "trigger_eval_interval": self.TRIGGER_EVAL_INTERVAL,
            "watchdog_interval": self.WATCHDOG_INTERVAL,
            "dispatcher_poll_interval": self.DISPATCHER_POLL_INTERVAL,
            "http_timeouts": {
                "fast": self.HTTP_TIMEOUT_FAST,
                "normal": self.HTTP_TIMEOUT_NORMAL,
                "slow": self.HTTP_TIMEOUT_SLOW,
            },
            "shell_timeouts": {
                "fast": self.SHELL_TIMEOUT_FAST,
                "normal": self.SHELL_TIMEOUT_NORMAL,
                "slow": self.SHELL_TIMEOUT_SLOW,
            },
            "retries": {
                "default": self.DEFAULT_RETRIES,
                "backoff_base": self.RETRY_BACKOFF_BASE,
                "backoff_max": self.RETRY_BACKOFF_MAX,
            },
            "concurrency": {
                "executor_pool": self.EXECUTOR_POOL_SIZE,
                "max_concurrent_pots": self.MAX_CONCURRENT_POTS,
            },
        }


# Instancia global de configuración turbo
TURBO = TurboConfig()


# ============================================================================
# CONFIGURACIONES PREDEFINIDAS
# ============================================================================

# Configuración TURBO (máxima velocidad)
TURBO_MAX_SPEED = TurboConfig(
    HEALTH_CHECK_INTERVAL=3.0,
    TRIGGER_EVAL_INTERVAL=1.0,
    WATCHDOG_INTERVAL=2.0,
    DISPATCHER_POLL_INTERVAL=0.05,  # 50ms
    AUTO_COMMIT_COOLDOWN=30,
    AUTO_REPAIR_COOLDOWN=15,
)

# Configuración BALANCED (equilibrio)
BALANCED = TurboConfig(
    HEALTH_CHECK_INTERVAL=10.0,
    TRIGGER_EVAL_INTERVAL=5.0,
    WATCHDOG_INTERVAL=5.0,
    DISPATCHER_POLL_INTERVAL=0.2,
    AUTO_COMMIT_COOLDOWN=120,
    AUTO_REPAIR_COOLDOWN=60,
)

# Configuración ECO (bajo consumo)
ECO = TurboConfig(
    HEALTH_CHECK_INTERVAL=60.0,
    TRIGGER_EVAL_INTERVAL=30.0,
    WATCHDOG_INTERVAL=30.0,
    DISPATCHER_POLL_INTERVAL=1.0,
    AUTO_COMMIT_COOLDOWN=600,
    AUTO_REPAIR_COOLDOWN=300,
)


def get_config(mode: str = "turbo") -> TurboConfig:
    """
    Obtiene configuración según el modo.
    
    Args:
        mode: "turbo", "balanced", "eco"
    """
    configs = {
        "turbo": TURBO_MAX_SPEED,
        "balanced": BALANCED,
        "eco": ECO,
    }
    return configs.get(mode.lower(), TURBO)


__all__ = [
    "TurboConfig",
    "TURBO",
    "TURBO_MAX_SPEED",
    "BALANCED",
    "ECO",
    "get_config",
]
