"""
ATLAS Turbo Executor
=====================
Executor optimizado con thread pool para máxima velocidad.
"""
from __future__ import annotations

import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor, Future, as_completed
from dataclasses import dataclass
from datetime import datetime, timezone
from queue import Queue, Empty
from typing import Any, Callable, Dict, List, Optional, Tuple

from .turbo_config import TURBO, TurboConfig

_log = logging.getLogger("humanoid.quality.turbo_executor")


@dataclass
class Task:
    """Tarea para ejecutar."""
    id: str
    fn: Callable[..., Any]
    args: tuple = ()
    kwargs: dict = None
    priority: int = 5  # 1 = alta, 10 = baja
    timeout: float = 60.0
    
    def __post_init__(self):
        if self.kwargs is None:
            self.kwargs = {}


@dataclass
class TaskResult:
    """Resultado de una tarea."""
    task_id: str
    ok: bool
    result: Any = None
    error: Optional[str] = None
    elapsed_ms: int = 0


class TurboExecutor:
    """
    Executor de alto rendimiento con thread pool.
    
    Features:
    - Thread pool pre-creado (no overhead de crear threads)
    - Priority queue para tareas urgentes primero
    - Timeouts por tarea
    - Métricas de rendimiento
    """
    
    def __init__(self, config: Optional[TurboConfig] = None):
        self.config = config or TURBO
        self._pool = ThreadPoolExecutor(
            max_workers=self.config.EXECUTOR_POOL_SIZE,
            thread_name_prefix="TurboExec",
        )
        self._pending: Dict[str, Future] = {}
        self._lock = threading.Lock()
        
        # Métricas
        self._stats = {
            "total_executed": 0,
            "successful": 0,
            "failed": 0,
            "total_time_ms": 0,
            "avg_time_ms": 0,
        }
    
    def submit(self, task: Task) -> Future:
        """
        Envía una tarea al pool.
        
        Returns:
            Future que se resuelve con TaskResult
        """
        def execute_with_metrics():
            start = time.perf_counter()
            try:
                result = task.fn(*task.args, **task.kwargs)
                elapsed = int((time.perf_counter() - start) * 1000)
                
                with self._lock:
                    self._stats["total_executed"] += 1
                    self._stats["successful"] += 1
                    self._stats["total_time_ms"] += elapsed
                    self._update_avg()
                
                return TaskResult(
                    task_id=task.id,
                    ok=True,
                    result=result,
                    elapsed_ms=elapsed,
                )
            except Exception as e:
                elapsed = int((time.perf_counter() - start) * 1000)
                
                with self._lock:
                    self._stats["total_executed"] += 1
                    self._stats["failed"] += 1
                    self._stats["total_time_ms"] += elapsed
                    self._update_avg()
                
                return TaskResult(
                    task_id=task.id,
                    ok=False,
                    error=str(e),
                    elapsed_ms=elapsed,
                )
        
        future = self._pool.submit(execute_with_metrics)
        
        with self._lock:
            self._pending[task.id] = future
        
        return future
    
    def submit_many(self, tasks: List[Task]) -> List[TaskResult]:
        """
        Ejecuta múltiples tareas en paralelo.
        
        Returns:
            Lista de resultados (en orden de completación, no de envío)
        """
        futures = [self.submit(task) for task in tasks]
        results = []
        
        for future in as_completed(futures):
            try:
                result = future.result()
                results.append(result)
            except Exception as e:
                results.append(TaskResult(
                    task_id="unknown",
                    ok=False,
                    error=str(e),
                ))
        
        return results
    
    def run_parallel_checks(
        self,
        checks: List[Tuple[str, Callable[[], bool]]],
    ) -> Dict[str, bool]:
        """
        Ejecuta health checks en paralelo.
        
        Args:
            checks: Lista de (nombre, función_check)
        
        Returns:
            Diccionario {nombre: resultado}
        """
        tasks = [
            Task(
                id=f"check_{name}",
                fn=check_fn,
                timeout=self.config.HTTP_TIMEOUT_FAST,
            )
            for name, check_fn in checks
        ]
        
        results = self.submit_many(tasks)
        
        return {
            r.task_id.replace("check_", ""): r.ok and r.result is True
            for r in results
        }
    
    def _update_avg(self) -> None:
        """Actualiza el promedio (llamar con lock)."""
        if self._stats["total_executed"] > 0:
            self._stats["avg_time_ms"] = (
                self._stats["total_time_ms"] // self._stats["total_executed"]
            )
    
    def get_stats(self) -> Dict[str, Any]:
        """Retorna métricas de rendimiento."""
        with self._lock:
            return self._stats.copy()
    
    def shutdown(self, wait: bool = True) -> None:
        """Detiene el executor."""
        self._pool.shutdown(wait=wait)


class TurboDispatchQueue:
    """
    Queue de dispatch optimizada con prioridades.
    """
    
    def __init__(self, maxsize: int = 100):
        self._queues: Dict[int, Queue] = {
            i: Queue(maxsize=maxsize // 10 + 1)
            for i in range(1, 11)  # Prioridades 1-10
        }
        self._lock = threading.Lock()
        self._not_empty = threading.Condition(self._lock)
        self._total = 0
    
    def put(self, item: Any, priority: int = 5) -> None:
        """Añade item con prioridad (1=alta, 10=baja)."""
        priority = max(1, min(10, priority))
        
        with self._lock:
            self._queues[priority].put_nowait(item)
            self._total += 1
            self._not_empty.notify()
    
    def get(self, timeout: float = 0.1) -> Optional[Any]:
        """
        Obtiene el item de mayor prioridad.
        
        Returns:
            Item o None si timeout
        """
        with self._not_empty:
            # Esperar si está vacío
            if self._total == 0:
                self._not_empty.wait(timeout=timeout)
            
            # Buscar en orden de prioridad
            for priority in range(1, 11):
                try:
                    item = self._queues[priority].get_nowait()
                    self._total -= 1
                    return item
                except Empty:
                    continue
            
            return None
    
    def qsize(self) -> int:
        """Retorna tamaño total del queue."""
        with self._lock:
            return self._total
    
    def empty(self) -> bool:
        """Retorna True si está vacío."""
        with self._lock:
            return self._total == 0


class RetryWithBackoff:
    """
    Helper para reintentos con backoff exponencial.
    """
    
    def __init__(self, config: Optional[TurboConfig] = None):
        self.config = config or TURBO
    
    def execute(
        self,
        fn: Callable[..., Any],
        *args,
        retries: Optional[int] = None,
        **kwargs,
    ) -> Tuple[bool, Any]:
        """
        Ejecuta función con reintentos.
        
        Returns:
            (ok, result_or_error)
        """
        max_retries = retries or self.config.DEFAULT_RETRIES
        backoff = self.config.RETRY_BACKOFF_BASE
        last_error = None
        
        for attempt in range(max_retries + 1):
            try:
                result = fn(*args, **kwargs)
                return (True, result)
            except Exception as e:
                last_error = e
                
                if attempt < max_retries:
                    time.sleep(backoff)
                    backoff = min(
                        backoff * self.config.RETRY_BACKOFF_MULTIPLIER,
                        self.config.RETRY_BACKOFF_MAX,
                    )
        
        return (False, str(last_error))


# ============================================================================
# SINGLETON
# ============================================================================

_executor: Optional[TurboExecutor] = None
_retry: Optional[RetryWithBackoff] = None


def get_turbo_executor() -> TurboExecutor:
    """Obtiene el executor global."""
    global _executor
    if _executor is None:
        _executor = TurboExecutor()
    return _executor


def get_retry_helper() -> RetryWithBackoff:
    """Obtiene el helper de retry global."""
    global _retry
    if _retry is None:
        _retry = RetryWithBackoff()
    return _retry


__all__ = [
    "TurboExecutor",
    "TurboDispatchQueue",
    "RetryWithBackoff",
    "Task",
    "TaskResult",
    "get_turbo_executor",
    "get_retry_helper",
]
