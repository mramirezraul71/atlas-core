"""
Libro de Vida (Book of Life) — Motor central de experiencias estructuradas.

Cada episodio almacena la cadena completa:
  contexto → percepciones → acciones → resultados → feedback → lecciones

El Libro integra las memorias existentes de ATLAS (Lifelog, Episodic, World Model,
Autobiographical) y añade su propia capa de episodios enriquecidos con búsqueda
por similitud textual y por tipo de tarea.
"""
from __future__ import annotations

import hashlib
import json
import logging
import os
import sqlite3
import time
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

log = logging.getLogger("atlas.libro_vida")

_DB_PATH: Optional[str] = None
_INSTANCE: Optional["LibroDeVida"] = None


def _default_db_path() -> str:
    base = os.getenv("ATLAS_DATA_DIR") or os.path.join(
        os.getenv("ATLAS_REPO_PATH", "C:\\ATLAS_PUSH"), "logs"
    )
    Path(base).mkdir(parents=True, exist_ok=True)
    return os.path.join(base, "libro_vida.sqlite")


# ── Dataclasses ──────────────────────────────────────────────────────────

@dataclass
class Percepcion:
    sensores: Dict[str, Any] = field(default_factory=dict)
    eventos: List[str] = field(default_factory=list)
    anomalias: List[str] = field(default_factory=list)

@dataclass
class AccionEjecutada:
    paso: int = 0
    descripcion: str = ""
    parametros: Dict[str, Any] = field(default_factory=dict)
    alternativas_descartadas: List[str] = field(default_factory=list)
    duracion_ms: float = 0.0
    exitoso: bool = True

@dataclass
class Resultado:
    exito: bool = True
    metricas: Dict[str, float] = field(default_factory=dict)
    eventos_riesgo: List[str] = field(default_factory=list)
    colisiones_evitadas: int = 0
    colisiones_producidas: int = 0

@dataclass
class Feedback:
    correcciones_humanas: List[str] = field(default_factory=list)
    recompensa: float = 0.0
    penalizacion: float = 0.0
    comentarios: List[str] = field(default_factory=list)
    sentiment: float = 0.0

@dataclass
class Leccion:
    texto: str = ""
    reglas_numericas: Dict[str, float] = field(default_factory=dict)
    recomendaciones: List[str] = field(default_factory=list)

@dataclass
class EpisodioVida:
    id: str = ""
    timestamp: float = 0.0
    tipo_tarea: str = ""
    objetivo: str = ""
    contexto_entorno: str = ""
    restricciones_seguridad: List[str] = field(default_factory=list)
    participantes: List[str] = field(default_factory=list)
    percepciones: Percepcion = field(default_factory=Percepcion)
    acciones: List[AccionEjecutada] = field(default_factory=list)
    resultado: Resultado = field(default_factory=Resultado)
    feedback: Feedback = field(default_factory=Feedback)
    lecciones: Leccion = field(default_factory=Leccion)
    importancia: float = 0.5
    tags: List[str] = field(default_factory=list)
    episode_ids_relacionados: List[str] = field(default_factory=list)
    valor_aprendizaje: str = "normal"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "EpisodioVida":
        ep = cls()
        for k, v in d.items():
            if k == "percepciones" and isinstance(v, dict):
                ep.percepciones = Percepcion(**v)
            elif k == "acciones" and isinstance(v, list):
                ep.acciones = [AccionEjecutada(**a) if isinstance(a, dict) else a for a in v]
            elif k == "resultado" and isinstance(v, dict):
                ep.resultado = Resultado(**v)
            elif k == "feedback" and isinstance(v, dict):
                ep.feedback = Feedback(**v)
            elif k == "lecciones" and isinstance(v, dict):
                ep.lecciones = Leccion(**v)
            elif hasattr(ep, k):
                setattr(ep, k, v)
        return ep


# ── Motor principal ──────────────────────────────────────────────────────

TASK_TYPES = [
    "transporte_objeto", "navegacion", "escaleras", "manipulacion",
    "interaccion_humana", "inspeccion", "limpieza", "apertura_puerta",
    "carga_pesada", "objeto_fragil", "patrulla", "emergencia",
    "mantenimiento", "comunicacion", "diagnostico", "otro",
]


class LibroDeVida:
    """Motor de persistencia y consulta del Libro de Vida de ATLAS."""

    def __init__(self, db_path: Optional[str] = None):
        self._db_path = db_path or _default_db_path()
        self._conn = self._init_db()
        log.info("LibroDeVida inicializado: %s", self._db_path)

    def _init_db(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self._db_path, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA busy_timeout=5000")
        conn.executescript("""
            CREATE TABLE IF NOT EXISTS episodios (
                id              TEXT PRIMARY KEY,
                timestamp_ts    REAL NOT NULL,
                tipo_tarea      TEXT NOT NULL,
                objetivo        TEXT NOT NULL,
                contexto_entorno TEXT DEFAULT '',
                restricciones   TEXT DEFAULT '[]',
                participantes   TEXT DEFAULT '[]',
                percepciones    TEXT DEFAULT '{}',
                acciones        TEXT DEFAULT '[]',
                resultado       TEXT DEFAULT '{}',
                feedback        TEXT DEFAULT '{}',
                lecciones       TEXT DEFAULT '{}',
                importancia     REAL DEFAULT 0.5,
                tags            TEXT DEFAULT '[]',
                episode_ids_rel TEXT DEFAULT '[]',
                valor_aprendizaje TEXT DEFAULT 'normal',
                exito           INTEGER DEFAULT 1,
                reward_total    REAL DEFAULT 0.0,
                search_text     TEXT DEFAULT '',
                created_at      TEXT DEFAULT (datetime('now'))
            );

            CREATE INDEX IF NOT EXISTS idx_ep_ts ON episodios(timestamp_ts);
            CREATE INDEX IF NOT EXISTS idx_ep_tipo ON episodios(tipo_tarea);
            CREATE INDEX IF NOT EXISTS idx_ep_exito ON episodios(exito);
            CREATE INDEX IF NOT EXISTS idx_ep_importancia ON episodios(importancia DESC);
            CREATE INDEX IF NOT EXISTS idx_ep_valor ON episodios(valor_aprendizaje);

            CREATE TABLE IF NOT EXISTS reglas_aprendidas (
                id          TEXT PRIMARY KEY,
                episodio_id TEXT NOT NULL,
                tipo_tarea  TEXT NOT NULL,
                regla       TEXT NOT NULL,
                valor       REAL,
                unidad      TEXT DEFAULT '',
                confianza   REAL DEFAULT 0.5,
                veces_validada INTEGER DEFAULT 1,
                created_at  TEXT DEFAULT (datetime('now')),
                FOREIGN KEY (episodio_id) REFERENCES episodios(id)
            );

            CREATE INDEX IF NOT EXISTS idx_reglas_tipo ON reglas_aprendidas(tipo_tarea);

            CREATE TABLE IF NOT EXISTS principios_generales (
                id          TEXT PRIMARY KEY,
                categoria   TEXT NOT NULL,
                principio   TEXT NOT NULL,
                fuente_episodios TEXT DEFAULT '[]',
                confianza   REAL DEFAULT 0.5,
                aplicaciones INTEGER DEFAULT 1,
                updated_at  TEXT DEFAULT (datetime('now'))
            );
        """)
        conn.commit()
        return conn

    def _gen_id(self) -> str:
        return "lv_" + uuid.uuid4().hex[:12]

    def _build_search_text(self, ep: EpisodioVida) -> str:
        parts = [
            ep.tipo_tarea, ep.objetivo, ep.contexto_entorno,
            " ".join(ep.tags),
            " ".join(ep.restricciones_seguridad),
            ep.lecciones.texto,
            " ".join(ep.lecciones.recomendaciones),
            " ".join(ep.percepciones.anomalias),
            " ".join(ep.percepciones.eventos),
            " ".join(a.descripcion for a in ep.acciones),
            " ".join(ep.resultado.eventos_riesgo),
            " ".join(ep.feedback.comentarios),
            " ".join(ep.feedback.correcciones_humanas),
        ]
        return " ".join(p for p in parts if p).lower()

    # ── CRUD ─────────────────────────────────────────────────────────────

    def registrar_episodio(self, ep: EpisodioVida) -> str:
        if not ep.id:
            ep.id = self._gen_id()
        if ep.timestamp == 0:
            ep.timestamp = time.time()
        search_text = self._build_search_text(ep)
        reward = ep.feedback.recompensa - ep.feedback.penalizacion

        self._conn.execute("""
            INSERT OR REPLACE INTO episodios
            (id, timestamp_ts, tipo_tarea, objetivo, contexto_entorno,
             restricciones, participantes, percepciones, acciones,
             resultado, feedback, lecciones, importancia, tags,
             episode_ids_rel, valor_aprendizaje, exito, reward_total, search_text)
            VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
        """, (
            ep.id, ep.timestamp, ep.tipo_tarea, ep.objetivo, ep.contexto_entorno,
            json.dumps(ep.restricciones_seguridad, ensure_ascii=False),
            json.dumps(ep.participantes, ensure_ascii=False),
            json.dumps(asdict(ep.percepciones), ensure_ascii=False),
            json.dumps([asdict(a) for a in ep.acciones], ensure_ascii=False),
            json.dumps(asdict(ep.resultado), ensure_ascii=False),
            json.dumps(asdict(ep.feedback), ensure_ascii=False),
            json.dumps(asdict(ep.lecciones), ensure_ascii=False),
            ep.importancia,
            json.dumps(ep.tags, ensure_ascii=False),
            json.dumps(ep.episode_ids_relacionados, ensure_ascii=False),
            ep.valor_aprendizaje,
            1 if ep.resultado.exito else 0,
            reward,
            search_text,
        ))
        self._conn.commit()

        for regla_key, regla_val in ep.lecciones.reglas_numericas.items():
            self._conn.execute("""
                INSERT OR REPLACE INTO reglas_aprendidas (id, episodio_id, tipo_tarea, regla, valor, confianza)
                VALUES (?, ?, ?, ?, ?, ?)
            """, (self._gen_id(), ep.id, ep.tipo_tarea, regla_key, regla_val, 0.5 + ep.importancia * 0.5))
            self._conn.commit()

        self._sync_to_subsystems(ep)
        log.info("Episodio registrado: %s [%s] %s", ep.id, ep.tipo_tarea, ep.objetivo[:60])
        return ep.id

    def _sync_to_subsystems(self, ep: EpisodioVida):
        """Propaga el episodio a Lifelog, World Model y Autobiographical si aplica."""
        try:
            from modules.humanoid.memory_engine.lifelog import get_lifelog
            ll = get_lifelog()
            ll.log(
                event_type="libro_vida_episode",
                source="libro_vida",
                perception=ep.objetivo,
                action=" → ".join(a.descripcion for a in ep.acciones[:5]),
                outcome="exito" if ep.resultado.exito else "fallo",
                success=ep.resultado.exito,
                reward=ep.feedback.recompensa - ep.feedback.penalizacion,
                importance=ep.importancia,
                tags=ep.tags,
                episode_id=ep.id,
                context={"tipo_tarea": ep.tipo_tarea, "entorno": ep.contexto_entorno},
            )
        except Exception as e:
            log.debug("Lifelog sync: %s", e)

        try:
            from modules.humanoid.world_model.engine import get_world_model
            wm = get_world_model()
            wm.record_outcome(
                action_type=ep.tipo_tarea,
                context={"objetivo": ep.objetivo, "entorno": ep.contexto_entorno},
                predicted="exito", actual="exito" if ep.resultado.exito else "fallo",
                success=ep.resultado.exito,
                duration_ms=sum(a.duracion_ms for a in ep.acciones),
            )
        except Exception as e:
            log.debug("WorldModel sync: %s", e)

        if ep.importancia >= 0.8 or not ep.resultado.exito:
            try:
                from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
                am = get_autobiographical_memory()
                cat = "achievement" if ep.resultado.exito else "failure"
                am.record_milestone(
                    title=f"[{ep.tipo_tarea}] {ep.objetivo[:80]}",
                    description=ep.lecciones.texto or f"Resultado: {'éxito' if ep.resultado.exito else 'fallo'}",
                    category=cat, importance=ep.importancia,
                    episode_ids=[ep.id],
                )
            except Exception as e:
                log.debug("Autobiographical sync: %s", e)

    def obtener_episodio(self, ep_id: str) -> Optional[EpisodioVida]:
        row = self._conn.execute("SELECT * FROM episodios WHERE id=?", (ep_id,)).fetchone()
        if not row:
            return None
        return self._row_to_episodio(row)

    def _row_to_episodio(self, row) -> EpisodioVida:
        cols = [d[0] for d in self._conn.execute("SELECT * FROM episodios LIMIT 0").description]
        d = dict(zip(cols, row))
        ep = EpisodioVida()
        ep.id = d["id"]
        ep.timestamp = d["timestamp_ts"]
        ep.tipo_tarea = d["tipo_tarea"]
        ep.objetivo = d["objetivo"]
        ep.contexto_entorno = d.get("contexto_entorno", "")
        ep.restricciones_seguridad = json.loads(d.get("restricciones", "[]"))
        ep.participantes = json.loads(d.get("participantes", "[]"))
        ep.percepciones = Percepcion(**json.loads(d.get("percepciones", "{}")))
        ep.acciones = [AccionEjecutada(**a) for a in json.loads(d.get("acciones", "[]"))]
        ep.resultado = Resultado(**json.loads(d.get("resultado", "{}")))
        ep.feedback = Feedback(**json.loads(d.get("feedback", "{}")))
        ep.lecciones = Leccion(**json.loads(d.get("lecciones", "{}")))
        ep.importancia = d.get("importancia", 0.5)
        ep.tags = json.loads(d.get("tags", "[]"))
        ep.episode_ids_relacionados = json.loads(d.get("episode_ids_rel", "[]"))
        ep.valor_aprendizaje = d.get("valor_aprendizaje", "normal")
        return ep

    # ── Búsqueda ─────────────────────────────────────────────────────────

    def buscar_similares(
        self,
        tipo_tarea: Optional[str] = None,
        objetivo: str = "",
        contexto: str = "",
        tags: Optional[List[str]] = None,
        limit: int = 5,
        solo_exitosos: Optional[bool] = None,
    ) -> List[EpisodioVida]:
        """Busca episodios similares por tipo de tarea, texto y tags."""
        conditions = []
        params: list = []

        if tipo_tarea:
            conditions.append("tipo_tarea = ?")
            params.append(tipo_tarea)

        if solo_exitosos is True:
            conditions.append("exito = 1")
        elif solo_exitosos is False:
            conditions.append("exito = 0")

        search_terms = " ".join(filter(None, [objetivo, contexto, " ".join(tags or [])])).lower().split()
        for term in search_terms[:8]:
            conditions.append("search_text LIKE ?")
            params.append(f"%{term}%")

        where = " AND ".join(conditions) if conditions else "1=1"
        query = f"""
            SELECT * FROM episodios
            WHERE {where}
            ORDER BY importancia DESC, timestamp_ts DESC
            LIMIT ?
        """
        params.append(limit)

        rows = self._conn.execute(query, params).fetchall()
        return [self._row_to_episodio(r) for r in rows]

    def buscar_por_texto(self, query: str, limit: int = 5) -> List[EpisodioVida]:
        terms = query.lower().split()[:10]
        if not terms:
            return []
        conditions = [f"search_text LIKE ?" for _ in terms]
        params = [f"%{t}%" for t in terms]
        sql = f"""
            SELECT * FROM episodios
            WHERE {" OR ".join(conditions)}
            ORDER BY importancia DESC, timestamp_ts DESC
            LIMIT ?
        """
        params.append(limit)
        rows = self._conn.execute(sql, params).fetchall()
        return [self._row_to_episodio(r) for r in rows]

    def obtener_reglas(self, tipo_tarea: Optional[str] = None, limit: int = 20) -> List[Dict]:
        if tipo_tarea:
            rows = self._conn.execute(
                "SELECT * FROM reglas_aprendidas WHERE tipo_tarea=? ORDER BY confianza DESC LIMIT ?",
                (tipo_tarea, limit)
            ).fetchall()
        else:
            rows = self._conn.execute(
                "SELECT * FROM reglas_aprendidas ORDER BY confianza DESC LIMIT ?", (limit,)
            ).fetchall()
        cols = [d[0] for d in self._conn.execute("SELECT * FROM reglas_aprendidas LIMIT 0").description]
        return [dict(zip(cols, r)) for r in rows]

    def obtener_principios(self, categoria: Optional[str] = None) -> List[Dict]:
        if categoria:
            rows = self._conn.execute(
                "SELECT * FROM principios_generales WHERE categoria=? ORDER BY confianza DESC", (categoria,)
            ).fetchall()
        else:
            rows = self._conn.execute(
                "SELECT * FROM principios_generales ORDER BY confianza DESC"
            ).fetchall()
        cols = [d[0] for d in self._conn.execute("SELECT * FROM principios_generales LIMIT 0").description]
        return [dict(zip(cols, r)) for r in rows]

    def registrar_principio(self, categoria: str, principio: str, episodio_ids: List[str] = None, confianza: float = 0.5) -> str:
        pid = "pr_" + uuid.uuid4().hex[:10]
        self._conn.execute("""
            INSERT INTO principios_generales (id, categoria, principio, fuente_episodios, confianza)
            VALUES (?, ?, ?, ?, ?)
        """, (pid, categoria, principio, json.dumps(episodio_ids or []), confianza))
        self._conn.commit()
        return pid

    # ── Estadísticas ─────────────────────────────────────────────────────

    def get_stats(self) -> Dict[str, Any]:
        total = self._conn.execute("SELECT COUNT(*) FROM episodios").fetchone()[0]
        exitos = self._conn.execute("SELECT COUNT(*) FROM episodios WHERE exito=1").fetchone()[0]
        fallos = self._conn.execute("SELECT COUNT(*) FROM episodios WHERE exito=0").fetchone()[0]
        alto_valor = self._conn.execute("SELECT COUNT(*) FROM episodios WHERE valor_aprendizaje='alto'").fetchone()[0]
        reglas = self._conn.execute("SELECT COUNT(*) FROM reglas_aprendidas").fetchone()[0]
        principios = self._conn.execute("SELECT COUNT(*) FROM principios_generales").fetchone()[0]

        tipos_row = self._conn.execute(
            "SELECT tipo_tarea, COUNT(*) as c FROM episodios GROUP BY tipo_tarea ORDER BY c DESC LIMIT 10"
        ).fetchall()
        por_tipo = {r[0]: r[1] for r in tipos_row}

        return {
            "total_episodios": total,
            "exitos": exitos,
            "fallos": fallos,
            "tasa_exito": round(exitos / total, 4) if total > 0 else 0.0,
            "alto_valor_aprendizaje": alto_valor,
            "reglas_aprendidas": reglas,
            "principios_generales": principios,
            "por_tipo_tarea": por_tipo,
        }

    def listar_recientes(self, limit: int = 10) -> List[Dict]:
        rows = self._conn.execute(
            "SELECT id, tipo_tarea, objetivo, exito, importancia, timestamp_ts FROM episodios ORDER BY timestamp_ts DESC LIMIT ?",
            (limit,)
        ).fetchall()
        return [{"id": r[0], "tipo_tarea": r[1], "objetivo": r[2], "exito": bool(r[3]), "importancia": r[4], "timestamp": r[5]} for r in rows]


def get_libro_vida() -> LibroDeVida:
    global _INSTANCE
    if _INSTANCE is None:
        _INSTANCE = LibroDeVida()
    return _INSTANCE
