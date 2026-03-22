"""Módulo 6A — Agente de Memoria Episódica de Errores.

Cada error (OCR fallido, API disconnect, slippage, drawdown) se guarda como:
  - Vector embedding (sentence-transformers)
  - Metadata: tipo, timestamp, contexto, solución aplicada, efectividad

Almacenamiento: ChromaDB (ya en el stack de Atlas).

Funciones principales:
  - record_error(): registra nuevo error + solución
  - find_similar(): busca errores similares en historial
  - get_repair_action(): retorna mejor acción de reparación conocida
  - learn(): actualiza efectividad de soluciones
"""

from __future__ import annotations

import json
import logging
import time
import uuid
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

logger = logging.getLogger("atlas.healing.error_agent")

try:
    import chromadb
    from chromadb.config import Settings as ChromaSettings
    _CHROMA_OK = True
except ImportError:
    _CHROMA_OK = False
    logger.warning("chromadb no disponible — memoria episódica en fallback JSON")

try:
    from sentence_transformers import SentenceTransformer
    _ST_OK = True
except ImportError:
    _ST_OK = False
    logger.warning("sentence-transformers no disponible — embeddings simplificados")


# ── Tipos de error conocidos ──────────────────────────────────────────────────

ERROR_TYPES = {
    "ocr_confidence_low":    "Precisión OCR por debajo del umbral",
    "stream_latency_high":   "Latencia WebSocket superior a 200ms",
    "api_disconnect":        "Desconexión de API Tradier",
    "order_rejected":        "Orden rechazada por Tradier",
    "drawdown_exceeded":     "Drawdown supera umbral configurado",
    "slippage_high":         "Slippage mayor al esperado",
    "regime_flip":           "Cambio brusco de régimen de mercado",
    "ros2_timeout":          "Timeout en comunicación ROS2",
    "camera_lost":           "Stream de cámara perdido",
    "memory_error":          "Error en acceso a ChromaDB/Redis",
}

# Acciones de reparación default por tipo de error
DEFAULT_REPAIRS: dict[str, str] = {
    "ocr_confidence_low":    "recalibrar_camara_ajustar_iluminacion",
    "stream_latency_high":   "reconectar_websocket_limpiar_buffer",
    "api_disconnect":        "reconectar_api_verificar_token",
    "order_rejected":        "verificar_pdt_reducir_cantidad",
    "drawdown_exceeded":     "activar_circuit_breaker_modo_defensivo",
    "slippage_high":         "usar_ordenes_limite_ajustar_horario",
    "regime_flip":           "cerrar_posiciones_esperar_confirmacion",
    "ros2_timeout":          "reiniciar_ros2_node",
    "camera_lost":           "reconectar_rtmp_verificar_red",
    "memory_error":          "reiniciar_chroma_redis_verificar_disco",
}


@dataclass
class ErrorRecord:
    """Registro de un error con contexto y solución."""
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    error_type: str = ""
    description: str = ""
    context: dict = field(default_factory=dict)
    repair_action: str = ""
    repair_effective: bool = False
    repair_time_s: float = 0.0
    timestamp: float = field(default_factory=time.time)
    system_state: dict = field(default_factory=dict)


@dataclass
class RepairAction:
    """Acción de reparación con metadata de éxito."""
    action: str
    confidence: float       # 0-1 basado en historial de efectividad
    source: str             # "memory" | "default" | "llm"
    similar_error_id: str = ""
    estimated_time_s: float = 30.0


class ErrorMemoryAgent:
    """Agente de memoria episódica para auto-aprendizaje de errores.

    Usa ChromaDB como vector store y sentence-transformers para embeddings.
    Fallback a JSON si ChromaDB no está disponible.

    Uso::

        agent = ErrorMemoryAgent()
        agent.initialize()

        # Registrar error
        record = agent.record_error(
            error_type="api_disconnect",
            description="WebSocket cerrado inesperadamente",
            context={"retries": 3, "last_symbol": "AAPL"},
        )

        # Buscar solución
        repair = agent.get_repair_action("api_disconnect", "WebSocket cerrado")
        print(repair.action, repair.confidence)

        # Marcar efectividad
        agent.mark_repair_result(record.id, effective=True, time_s=12.5)
    """

    COLLECTION_NAME = "atlas_error_memory"
    EMBEDDING_MODEL = "all-MiniLM-L6-v2"   # pequeño, bueno para Jetson
    FALLBACK_PATH   = Path("data/operation/error_memory.jsonl")
    MIN_SIMILARITY  = 0.75   # umbral para considerar error "similar"

    def __init__(self, chroma_path: str = "data/chroma_errors") -> None:
        self.chroma_path = chroma_path
        self._client: Optional[object] = None
        self._collection: Optional[object] = None
        self._embedder: Optional[object] = None
        self._records: list[ErrorRecord] = []  # fallback en memoria

    def initialize(self) -> None:
        """Inicializa ChromaDB y modelo de embeddings."""
        self._init_embedder()
        self._init_chroma()
        self._load_fallback()
        logger.info("ErrorMemoryAgent inicializado (%s records en memoria)",
                    len(self._records))

    def _init_embedder(self) -> None:
        if not _ST_OK:
            return
        try:
            self._embedder = SentenceTransformer(self.EMBEDDING_MODEL)
            logger.info("SentenceTransformer cargado: %s", self.EMBEDDING_MODEL)
        except Exception as exc:
            logger.warning("Error cargando embedder: %s", exc)

    def _init_chroma(self) -> None:
        if not _CHROMA_OK:
            return
        try:
            Path(self.chroma_path).mkdir(parents=True, exist_ok=True)
            self._client = chromadb.PersistentClient(path=self.chroma_path)  # type: ignore[attr-defined]
            self._collection = self._client.get_or_create_collection(  # type: ignore[union-attr]
                name=self.COLLECTION_NAME,
                metadata={"hnsw:space": "cosine"},
            )
            logger.info("ChromaDB inicializado: %d documentos", self._collection.count())  # type: ignore[union-attr]
        except Exception as exc:
            logger.error("Error inicializando ChromaDB: %s", exc)

    def _load_fallback(self) -> None:
        """Carga historial JSON si ChromaDB no disponible."""
        if not self.FALLBACK_PATH.exists():
            return
        try:
            for line in self.FALLBACK_PATH.read_text().splitlines():
                if line.strip():
                    data = json.loads(line)
                    self._records.append(ErrorRecord(**data))
        except Exception as exc:
            logger.warning("Error cargando fallback: %s", exc)

    # ── Registro de errores ───────────────────────────────────────────────────

    def record_error(
        self,
        error_type: str,
        description: str,
        context: dict | None = None,
        system_state: dict | None = None,
    ) -> ErrorRecord:
        """Registra un nuevo error y lo vectoriza en ChromaDB."""
        repair_action = DEFAULT_REPAIRS.get(error_type, "investigar_manualmente")
        record = ErrorRecord(
            error_type    = error_type,
            description   = description,
            context       = context or {},
            repair_action = repair_action,
            system_state  = system_state or {},
        )

        # Persistir en ChromaDB
        text = f"{error_type}: {description}"
        embedding = self._embed(text)

        if self._collection is not None and embedding is not None:
            try:
                self._collection.add(  # type: ignore[union-attr]
                    ids        = [record.id],
                    embeddings = [embedding],
                    documents  = [text],
                    metadatas  = [{
                        "error_type":      record.error_type,
                        "repair_action":   record.repair_action,
                        "repair_effective": False,
                        "repair_time_s":   0.0,
                        "timestamp":       record.timestamp,
                    }],
                )
            except Exception as exc:
                logger.error("ChromaDB add error: %s", exc)

        # Persistir en fallback JSON
        self._records.append(record)
        self._save_fallback(record)

        logger.warning("ERROR [%s] %s → acción: %s", error_type, description, repair_action)
        return record

    # ── Búsqueda de errores similares ────────────────────────────────────────

    def find_similar(
        self, description: str, n_results: int = 5
    ) -> list[dict]:
        """Busca errores similares en ChromaDB por similitud semántica."""
        if self._collection is None:
            return self._fallback_search(description, n_results)

        embedding = self._embed(description)
        if embedding is None:
            return []

        try:
            results = self._collection.query(  # type: ignore[union-attr]
                query_embeddings=[embedding],
                n_results=min(n_results, max(1, self._collection.count())),  # type: ignore[union-attr]
                include=["documents", "metadatas", "distances"],
            )
            similar = []
            ids       = results.get("ids", [[]])[0]
            docs      = results.get("documents", [[]])[0]
            metas     = results.get("metadatas", [[]])[0]
            distances = results.get("distances", [[]])[0]

            for doc_id, doc, meta, dist in zip(ids, docs, metas, distances):
                similarity = 1.0 - float(dist)   # cosine → similitud
                if similarity >= self.MIN_SIMILARITY:
                    similar.append({
                        "id":             doc_id,
                        "document":       doc,
                        "repair_action":  meta.get("repair_action", ""),
                        "effective":      meta.get("repair_effective", False),
                        "repair_time_s":  meta.get("repair_time_s", 30.0),
                        "similarity":     round(similarity, 3),
                    })
            return similar
        except Exception as exc:
            logger.error("Error en búsqueda ChromaDB: %s", exc)
            return []

    def _fallback_search(self, description: str, n: int) -> list[dict]:
        """Búsqueda por palabras clave cuando ChromaDB no está disponible."""
        words = set(description.lower().split())
        scored = []
        for r in self._records:
            doc_words = set(r.description.lower().split())
            overlap = len(words & doc_words) / max(len(words), 1)
            if overlap > 0.3:
                scored.append((overlap, r))
        scored.sort(key=lambda x: x[0], reverse=True)
        return [
            {
                "id":            r.id,
                "document":      r.description,
                "repair_action": r.repair_action,
                "effective":     r.repair_effective,
                "repair_time_s": r.repair_time_s,
                "similarity":    round(score, 3),
            }
            for score, r in scored[:n]
        ]

    # ── Obtener acción de reparación ──────────────────────────────────────────

    def get_repair_action(
        self, error_type: str, description: str
    ) -> RepairAction:
        """Retorna la mejor acción de reparación basada en historial."""
        # Buscar en memoria episódica
        similar = self.find_similar(description, n_results=5)

        if similar:
            # Priorizar acciones efectivas
            effective = [s for s in similar if s["effective"]]
            best = effective[0] if effective else similar[0]

            return RepairAction(
                action          = best["repair_action"],
                confidence      = best["similarity"],
                source          = "memory",
                similar_error_id= best["id"],
                estimated_time_s= best.get("repair_time_s", 30.0),
            )

        # Fallback a acción default
        default_action = DEFAULT_REPAIRS.get(error_type, "investigar_manualmente")
        return RepairAction(
            action          = default_action,
            confidence      = 0.5,
            source          = "default",
            estimated_time_s= 30.0,
        )

    # ── Marcar resultado de reparación ────────────────────────────────────────

    def mark_repair_result(
        self, record_id: str, effective: bool, time_s: float = 0.0
    ) -> None:
        """Actualiza efectividad de una reparación en ChromaDB."""
        # Actualizar en memoria local
        for r in self._records:
            if r.id == record_id:
                r.repair_effective = effective
                r.repair_time_s = time_s
                break

        # Actualizar en ChromaDB
        if self._collection is not None:
            try:
                self._collection.update(  # type: ignore[union-attr]
                    ids=       [record_id],
                    metadatas= [{"repair_effective": effective, "repair_time_s": time_s}],
                )
            except Exception as exc:
                logger.debug("Error actualizando ChromaDB: %s", exc)

        logger.info(
            "Reparación %s marcada como %s (%.1fs)",
            record_id, "EFECTIVA" if effective else "FALLIDA", time_s
        )

    # ── Estadísticas ──────────────────────────────────────────────────────────

    def stats(self) -> dict:
        total = len(self._records)
        effective = sum(1 for r in self._records if r.repair_effective)
        by_type: dict[str, int] = {}
        for r in self._records:
            by_type[r.error_type] = by_type.get(r.error_type, 0) + 1

        chroma_count = 0
        if self._collection is not None:
            try:
                chroma_count = self._collection.count()  # type: ignore[union-attr]
            except Exception:
                pass

        return {
            "total_errors":    total,
            "effective_repairs": effective,
            "effectiveness_rate": round(effective / total, 3) if total > 0 else 0.0,
            "by_type":         by_type,
            "chroma_documents": chroma_count,
        }

    # ── Utilidades ────────────────────────────────────────────────────────────

    def _embed(self, text: str) -> Optional[list[float]]:
        if self._embedder is None:
            # Fallback: hash simple del texto
            h = hash(text) % (10**8)
            return [float(b) / 255 for b in h.to_bytes(8, "big")]
        try:
            return self._embedder.encode(text).tolist()  # type: ignore[attr-defined, union-attr]
        except Exception as exc:
            logger.debug("Error embeddiendo texto: %s", exc)
            return None

    def _save_fallback(self, record: ErrorRecord) -> None:
        try:
            self.FALLBACK_PATH.parent.mkdir(parents=True, exist_ok=True)
            with open(self.FALLBACK_PATH, "a", encoding="utf-8") as f:
                f.write(json.dumps(asdict(record)) + "\n")
        except Exception as exc:
            logger.debug("Error guardando fallback: %s", exc)
