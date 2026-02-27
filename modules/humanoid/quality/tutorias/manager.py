"""
Gestor principal del sistema de Tutorías y Visitas
"""

import json
import os
import sqlite3
from datetime import datetime, timedelta
from typing import List, Optional, Dict, Any
from pathlib import Path

from .models import (
    Especialista, Visita, Informe, Recomendacion, 
    Evaluacion, SeguimientoMejora, TipoVisita,
    EstadoRecomendacion, NivelEvaluacion, PrioridadRecomendacion
)


class TutoriasManager:
    """
    Gestor central del sistema de tutorías y visitas.
    Maneja especialistas, visitas, informes y seguimientos.
    """
    
    def __init__(self, db_path: str = None):
        """Inicializa el gestor con la base de datos"""
        if db_path is None:
            base_dir = Path(__file__).parent.parent.parent.parent.parent
            data_dir = base_dir / "data" / "quality"
            data_dir.mkdir(parents=True, exist_ok=True)
            db_path = str(data_dir / "tutorias.db")
        
        self.db_path = db_path
        self._init_database()
        self._visita_actual: Optional[Visita] = None
    
    def _init_database(self):
        """Inicializa las tablas de la base de datos"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Tabla de especialistas
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS especialistas (
                id TEXT PRIMARY KEY,
                nombre TEXT NOT NULL,
                rol TEXT,
                especialidad TEXT,
                email TEXT,
                firma_digital TEXT,
                fecha_registro TEXT,
                visitas_realizadas INTEGER DEFAULT 0,
                data_json TEXT
            )
        """)
        
        # Tabla de visitas
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS visitas (
                id TEXT PRIMARY KEY,
                tipo TEXT,
                especialista_id TEXT,
                fecha_inicio TEXT,
                fecha_fin TEXT,
                duracion_minutos INTEGER,
                motivo TEXT,
                modulos_revisados TEXT,
                objetivos TEXT,
                completada INTEGER DEFAULT 0,
                version_atlas TEXT,
                ambiente TEXT,
                notas TEXT,
                data_json TEXT,
                FOREIGN KEY (especialista_id) REFERENCES especialistas(id)
            )
        """)
        
        # Tabla de informes
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS informes (
                id TEXT PRIMARY KEY,
                visita_id TEXT,
                titulo TEXT,
                resumen TEXT,
                contenido TEXT,
                puntuacion_global REAL,
                firmado INTEGER DEFAULT 0,
                firma_especialista TEXT,
                fecha_firma TEXT,
                hash_verificacion TEXT,
                data_json TEXT,
                FOREIGN KEY (visita_id) REFERENCES visitas(id)
            )
        """)
        
        # Tabla de recomendaciones
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS recomendaciones (
                id TEXT PRIMARY KEY,
                informe_id TEXT,
                titulo TEXT,
                descripcion TEXT,
                modulo_afectado TEXT,
                prioridad TEXT,
                estado TEXT,
                fecha_creacion TEXT,
                fecha_limite TEXT,
                especialista_id TEXT,
                data_json TEXT,
                FOREIGN KEY (informe_id) REFERENCES informes(id)
            )
        """)
        
        # Tabla de seguimientos
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS seguimientos (
                id TEXT PRIMARY KEY,
                recomendacion_id TEXT,
                titulo TEXT,
                descripcion TEXT,
                fecha_inicio TEXT,
                fecha_objetivo TEXT,
                porcentaje_avance INTEGER,
                responsable TEXT,
                ultima_actualizacion TEXT,
                completado INTEGER DEFAULT 0,
                data_json TEXT,
                FOREIGN KEY (recomendacion_id) REFERENCES recomendaciones(id)
            )
        """)
        
        conn.commit()
        conn.close()
    
    # ==================== ESPECIALISTAS ====================
    
    def registrar_especialista(self, especialista: Especialista) -> str:
        """Registra un nuevo especialista en el sistema"""
        if not especialista.firma_digital:
            especialista.generar_firma()
        
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            INSERT OR REPLACE INTO especialistas 
            (id, nombre, rol, especialidad, email, firma_digital, fecha_registro, visitas_realizadas, data_json)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            especialista.id,
            especialista.nombre,
            especialista.rol,
            especialista.especialidad,
            especialista.email,
            especialista.firma_digital,
            especialista.fecha_registro.isoformat(),
            especialista.visitas_realizadas,
            json.dumps(especialista.to_dict())
        ))
        
        conn.commit()
        conn.close()
        return especialista.id
    
    def obtener_especialista(self, especialista_id: str) -> Optional[Especialista]:
        """Obtiene un especialista por ID"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("SELECT data_json FROM especialistas WHERE id = ?", (especialista_id,))
        row = cursor.fetchone()
        conn.close()
        
        if row:
            data = json.loads(row[0])
            return self._dict_to_especialista(data)
        return None
    
    def listar_especialistas(self) -> List[Especialista]:
        """Lista todos los especialistas registrados"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("SELECT data_json FROM especialistas ORDER BY nombre")
        rows = cursor.fetchall()
        conn.close()
        
        return [self._dict_to_especialista(json.loads(row[0])) for row in rows]
    
    def _dict_to_especialista(self, data: Dict) -> Especialista:
        """Convierte un diccionario a objeto Especialista"""
        esp = Especialista(
            id=data.get("id", ""),
            nombre=data.get("nombre", ""),
            rol=data.get("rol", ""),
            especialidad=data.get("especialidad", ""),
            email=data.get("email", ""),
            firma_digital=data.get("firma_digital", ""),
            visitas_realizadas=data.get("visitas_realizadas", 0)
        )
        if data.get("fecha_registro"):
            esp.fecha_registro = datetime.fromisoformat(data["fecha_registro"])
        return esp
    
    # ==================== VISITAS ====================
    
    def iniciar_visita(
        self,
        especialista: Especialista,
        tipo: TipoVisita = TipoVisita.TUTORIA,
        motivo: str = "",
        modulos: List[str] = None,
        objetivos: List[str] = None
    ) -> Visita:
        """Inicia una nueva visita de especialista"""
        visita = Visita(
            tipo=tipo,
            especialista=especialista,
            motivo=motivo,
            modulos_revisados=modulos or [],
            objetivos=objetivos or [],
            version_atlas=self._get_atlas_version()
        )
        
        self._visita_actual = visita
        self._guardar_visita(visita)
        
        # Incrementar contador de visitas del especialista
        especialista.visitas_realizadas += 1
        self.registrar_especialista(especialista)
        
        return visita
    
    def finalizar_visita(self, informe: Informe = None) -> Visita:
        """Finaliza la visita actual y guarda el informe"""
        if not self._visita_actual:
            raise ValueError("No hay visita en curso")
        
        self._visita_actual.finalizar()
        
        if informe:
            # Firmar el informe
            if self._visita_actual.especialista and not informe.firmado:
                informe.firmar(self._visita_actual.especialista)
            
            self._visita_actual.informe = informe
            self._guardar_informe(informe, self._visita_actual.id)
            
            # Guardar recomendaciones
            for rec in informe.recomendaciones:
                rec.especialista_id = self._visita_actual.especialista.id
                self._guardar_recomendacion(rec, informe.id)
        
        self._guardar_visita(self._visita_actual)
        
        visita_finalizada = self._visita_actual
        self._visita_actual = None
        
        return visita_finalizada
    
    def obtener_visita(self, visita_id: str) -> Optional[Visita]:
        """Obtiene una visita por ID"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("SELECT data_json FROM visitas WHERE id = ?", (visita_id,))
        row = cursor.fetchone()
        conn.close()
        
        if row:
            return self._dict_to_visita(json.loads(row[0]))
        return None
    
    def listar_visitas(
        self,
        limite: int = 50,
        tipo: TipoVisita = None,
        especialista_id: str = None,
        solo_completadas: bool = False
    ) -> List[Visita]:
        """Lista visitas con filtros opcionales"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        query = "SELECT data_json FROM visitas WHERE 1=1"
        params = []
        
        if tipo:
            query += " AND tipo = ?"
            params.append(tipo.value)
        
        if especialista_id:
            query += " AND especialista_id = ?"
            params.append(especialista_id)
        
        if solo_completadas:
            query += " AND completada = 1"
        
        query += " ORDER BY fecha_inicio DESC LIMIT ?"
        params.append(limite)
        
        cursor.execute(query, params)
        rows = cursor.fetchall()
        conn.close()
        
        return [self._dict_to_visita(json.loads(row[0])) for row in rows]
    
    def _guardar_visita(self, visita: Visita):
        """Guarda una visita en la base de datos"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            INSERT OR REPLACE INTO visitas
            (id, tipo, especialista_id, fecha_inicio, fecha_fin, duracion_minutos,
             motivo, modulos_revisados, objetivos, completada, version_atlas,
             ambiente, notas, data_json)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            visita.id,
            visita.tipo.value,
            visita.especialista.id if visita.especialista else None,
            visita.fecha_inicio.isoformat(),
            visita.fecha_fin.isoformat() if visita.fecha_fin else None,
            visita.duracion_minutos,
            visita.motivo,
            json.dumps(visita.modulos_revisados),
            json.dumps(visita.objetivos),
            1 if visita.completada else 0,
            visita.version_atlas,
            visita.ambiente,
            visita.notas,
            json.dumps(visita.to_dict())
        ))
        
        conn.commit()
        conn.close()
    
    def _dict_to_visita(self, data: Dict) -> Visita:
        """Convierte un diccionario a objeto Visita"""
        visita = Visita(
            id=data.get("id", ""),
            tipo=TipoVisita(data.get("tipo", "tutoria")),
            motivo=data.get("motivo", ""),
            modulos_revisados=data.get("modulos_revisados", []),
            objetivos=data.get("objetivos", []),
            completada=data.get("completada", False),
            version_atlas=data.get("version_atlas", ""),
            ambiente=data.get("ambiente", ""),
            notas=data.get("notas", "")
        )
        
        if data.get("fecha_inicio"):
            visita.fecha_inicio = datetime.fromisoformat(data["fecha_inicio"])
        if data.get("fecha_fin"):
            visita.fecha_fin = datetime.fromisoformat(data["fecha_fin"])
        if data.get("duracion_minutos"):
            visita.duracion_minutos = data["duracion_minutos"]
        if data.get("especialista"):
            visita.especialista = self._dict_to_especialista(data["especialista"])
        if data.get("informe"):
            visita.informe = self._dict_to_informe(data["informe"])
        
        return visita
    
    # ==================== INFORMES ====================
    
    def crear_informe(
        self,
        titulo: str,
        resumen: str,
        contenido: str,
        evaluaciones: List[Evaluacion] = None,
        recomendaciones: List[Recomendacion] = None
    ) -> Informe:
        """Crea un nuevo informe"""
        informe = Informe(
            titulo=titulo,
            resumen=resumen,
            contenido=contenido,
            evaluaciones=evaluaciones or [],
            recomendaciones=recomendaciones or []
        )
        return informe
    
    def obtener_informe(self, informe_id: str) -> Optional[Informe]:
        """Obtiene un informe por ID"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("SELECT data_json FROM informes WHERE id = ?", (informe_id,))
        row = cursor.fetchone()
        conn.close()
        
        if row:
            return self._dict_to_informe(json.loads(row[0]))
        return None
    
    def listar_informes(self, limite: int = 50, solo_firmados: bool = False) -> List[Informe]:
        """Lista informes"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        query = "SELECT data_json FROM informes"
        if solo_firmados:
            query += " WHERE firmado = 1"
        query += " ORDER BY fecha_firma DESC LIMIT ?"
        
        cursor.execute(query, (limite,))
        rows = cursor.fetchall()
        conn.close()
        
        return [self._dict_to_informe(json.loads(row[0])) for row in rows]
    
    def _guardar_informe(self, informe: Informe, visita_id: str):
        """Guarda un informe en la base de datos"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            INSERT OR REPLACE INTO informes
            (id, visita_id, titulo, resumen, contenido, puntuacion_global,
             firmado, firma_especialista, fecha_firma, hash_verificacion, data_json)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            informe.id,
            visita_id,
            informe.titulo,
            informe.resumen,
            informe.contenido,
            informe.calcular_puntuacion_global(),
            1 if informe.firmado else 0,
            informe.firma_especialista,
            informe.fecha_firma.isoformat() if informe.fecha_firma else None,
            informe.hash_verificacion,
            json.dumps(informe.to_dict())
        ))
        
        conn.commit()
        conn.close()
    
    def _dict_to_informe(self, data: Dict) -> Informe:
        """Convierte un diccionario a objeto Informe"""
        informe = Informe(
            id=data.get("id", ""),
            titulo=data.get("titulo", ""),
            resumen=data.get("resumen", ""),
            contenido=data.get("contenido", ""),
            objetivos_cumplidos=data.get("objetivos_cumplidos", []),
            objetivos_pendientes=data.get("objetivos_pendientes", []),
            archivos_adjuntos=data.get("archivos_adjuntos", []),
            codigo_revisado=data.get("codigo_revisado", []),
            metricas_capturadas=data.get("metricas_capturadas", {}),
            proximos_pasos=data.get("proximos_pasos", []),
            firmado=data.get("firmado", False),
            firma_especialista=data.get("firma_especialista", ""),
            hash_verificacion=data.get("hash_verificacion", "")
        )
        
        if data.get("fecha_firma"):
            informe.fecha_firma = datetime.fromisoformat(data["fecha_firma"])
        
        if data.get("evaluaciones"):
            informe.evaluaciones = [
                self._dict_to_evaluacion(e) for e in data["evaluaciones"]
            ]
        
        if data.get("recomendaciones"):
            informe.recomendaciones = [
                self._dict_to_recomendacion(r) for r in data["recomendaciones"]
            ]
        
        return informe
    
    def _dict_to_evaluacion(self, data: Dict) -> Evaluacion:
        """Convierte un diccionario a Evaluacion"""
        return Evaluacion(
            aspecto=data.get("aspecto", ""),
            nivel=NivelEvaluacion[data.get("nivel", "ACEPTABLE")],
            puntuacion=data.get("puntuacion", 3.0),
            comentario=data.get("comentario", ""),
            metricas=data.get("metricas", {})
        )
    
    # ==================== RECOMENDACIONES ====================
    
    def crear_recomendacion(
        self,
        titulo: str,
        descripcion: str,
        modulo_afectado: str,
        prioridad: PrioridadRecomendacion = PrioridadRecomendacion.MEDIA,
        pasos: List[str] = None
    ) -> Recomendacion:
        """Crea una nueva recomendación"""
        return Recomendacion(
            titulo=titulo,
            descripcion=descripcion,
            modulo_afectado=modulo_afectado,
            prioridad=prioridad,
            pasos_implementacion=pasos or []
        )
    
    def actualizar_estado_recomendacion(
        self,
        recomendacion_id: str,
        estado: EstadoRecomendacion,
        nota: str = None
    ):
        """Actualiza el estado de una recomendación"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("SELECT data_json FROM recomendaciones WHERE id = ?", (recomendacion_id,))
        row = cursor.fetchone()
        
        if row:
            data = json.loads(row[0])
            data["estado"] = estado.name
            if nota:
                if "notas_seguimiento" not in data:
                    data["notas_seguimiento"] = []
                data["notas_seguimiento"].append(f"[{datetime.now().isoformat()}] {nota}")
            
            cursor.execute(
                "UPDATE recomendaciones SET estado = ?, data_json = ? WHERE id = ?",
                (estado.name, json.dumps(data), recomendacion_id)
            )
            conn.commit()
        
        conn.close()
    
    def listar_recomendaciones(
        self,
        estado: EstadoRecomendacion = None,
        modulo: str = None,
        limite: int = 50
    ) -> List[Recomendacion]:
        """Lista recomendaciones con filtros"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        query = "SELECT data_json FROM recomendaciones WHERE 1=1"
        params = []
        
        if estado:
            query += " AND estado = ?"
            params.append(estado.name)
        
        if modulo:
            query += " AND modulo_afectado LIKE ?"
            params.append(f"%{modulo}%")
        
        query += " ORDER BY CASE prioridad "
        query += "WHEN 'CRITICA' THEN 1 WHEN 'ALTA' THEN 2 WHEN 'MEDIA' THEN 3 "
        query += "WHEN 'BAJA' THEN 4 ELSE 5 END LIMIT ?"
        params.append(limite)
        
        cursor.execute(query, params)
        rows = cursor.fetchall()
        conn.close()
        
        return [self._dict_to_recomendacion(json.loads(row[0])) for row in rows]
    
    def _guardar_recomendacion(self, rec: Recomendacion, informe_id: str):
        """Guarda una recomendación"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            INSERT OR REPLACE INTO recomendaciones
            (id, informe_id, titulo, descripcion, modulo_afectado, prioridad,
             estado, fecha_creacion, fecha_limite, especialista_id, data_json)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            rec.id,
            informe_id,
            rec.titulo,
            rec.descripcion,
            rec.modulo_afectado,
            rec.prioridad.name,
            rec.estado.name,
            rec.fecha_creacion.isoformat(),
            rec.fecha_limite.isoformat() if rec.fecha_limite else None,
            rec.especialista_id,
            json.dumps(rec.to_dict())
        ))
        
        conn.commit()
        conn.close()
    
    def _dict_to_recomendacion(self, data: Dict) -> Recomendacion:
        """Convierte un diccionario a Recomendacion"""
        rec = Recomendacion(
            id=data.get("id", ""),
            titulo=data.get("titulo", ""),
            descripcion=data.get("descripcion", ""),
            modulo_afectado=data.get("modulo_afectado", ""),
            prioridad=PrioridadRecomendacion[data.get("prioridad", "MEDIA")],
            estado=EstadoRecomendacion[data.get("estado", "PENDIENTE")],
            pasos_implementacion=data.get("pasos_implementacion", []),
            recursos_necesarios=data.get("recursos_necesarios", []),
            especialista_id=data.get("especialista_id", ""),
            notas_seguimiento=data.get("notas_seguimiento", [])
        )
        
        if data.get("fecha_creacion"):
            rec.fecha_creacion = datetime.fromisoformat(data["fecha_creacion"])
        if data.get("fecha_limite"):
            rec.fecha_limite = datetime.fromisoformat(data["fecha_limite"])
        
        return rec
    
    # ==================== SEGUIMIENTOS ====================
    
    def crear_seguimiento(
        self,
        recomendacion: Recomendacion,
        responsable: str = "ATLAS"
    ) -> SeguimientoMejora:
        """Crea un seguimiento para una recomendación"""
        seguimiento = SeguimientoMejora(
            recomendacion_id=recomendacion.id,
            titulo=f"Seguimiento: {recomendacion.titulo}",
            descripcion=recomendacion.descripcion,
            responsable=responsable
        )
        
        # Crear hitos basados en pasos de implementación
        for paso in recomendacion.pasos_implementacion:
            seguimiento.agregar_hito(paso, completado=False)
        
        self._guardar_seguimiento(seguimiento)
        return seguimiento
    
    def actualizar_seguimiento(
        self,
        seguimiento_id: str,
        hito_idx: int = None,
        completado: bool = True
    ):
        """Actualiza un seguimiento, marcando hitos como completados"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("SELECT data_json FROM seguimientos WHERE id = ?", (seguimiento_id,))
        row = cursor.fetchone()
        
        if row:
            data = json.loads(row[0])
            seg = SeguimientoMejora(
                id=data["id"],
                recomendacion_id=data.get("recomendacion_id", ""),
                titulo=data.get("titulo", ""),
                descripcion=data.get("descripcion", ""),
                hitos=data.get("hitos", []),
                responsable=data.get("responsable", "")
            )
            
            if hito_idx is not None and 0 <= hito_idx < len(seg.hitos):
                seg.hitos[hito_idx]["completado"] = completado
                seg._recalcular_avance()
            
            self._guardar_seguimiento(seg)
        
        conn.close()
    
    def listar_seguimientos(
        self,
        solo_activos: bool = True,
        limite: int = 50
    ) -> List[SeguimientoMejora]:
        """Lista seguimientos"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        query = "SELECT data_json FROM seguimientos"
        if solo_activos:
            query += " WHERE completado = 0"
        query += " ORDER BY ultima_actualizacion DESC LIMIT ?"
        
        cursor.execute(query, (limite,))
        rows = cursor.fetchall()
        conn.close()
        
        seguimientos = []
        for row in rows:
            data = json.loads(row[0])
            seg = SeguimientoMejora(
                id=data.get("id", ""),
                recomendacion_id=data.get("recomendacion_id", ""),
                titulo=data.get("titulo", ""),
                descripcion=data.get("descripcion", ""),
                porcentaje_avance=data.get("porcentaje_avance", 0),
                hitos=data.get("hitos", []),
                responsable=data.get("responsable", ""),
                completado=data.get("completado", False)
            )
            if data.get("fecha_inicio"):
                seg.fecha_inicio = datetime.fromisoformat(data["fecha_inicio"])
            if data.get("fecha_objetivo"):
                seg.fecha_objetivo = datetime.fromisoformat(data["fecha_objetivo"])
            if data.get("ultima_actualizacion"):
                seg.ultima_actualizacion = datetime.fromisoformat(data["ultima_actualizacion"])
            seguimientos.append(seg)
        
        return seguimientos
    
    def _guardar_seguimiento(self, seg: SeguimientoMejora):
        """Guarda un seguimiento"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("""
            INSERT OR REPLACE INTO seguimientos
            (id, recomendacion_id, titulo, descripcion, fecha_inicio,
             fecha_objetivo, porcentaje_avance, responsable, ultima_actualizacion,
             completado, data_json)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            seg.id,
            seg.recomendacion_id,
            seg.titulo,
            seg.descripcion,
            seg.fecha_inicio.isoformat(),
            seg.fecha_objetivo.isoformat() if seg.fecha_objetivo else None,
            seg.porcentaje_avance,
            seg.responsable,
            seg.ultima_actualizacion.isoformat(),
            1 if seg.completado else 0,
            json.dumps(seg.to_dict())
        ))
        
        conn.commit()
        conn.close()
    
    # ==================== ESTADÍSTICAS ====================
    
    def obtener_estadisticas(self) -> Dict[str, Any]:
        """Obtiene estadísticas generales del sistema"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        stats = {}
        
        # Total especialistas
        cursor.execute("SELECT COUNT(*) FROM especialistas")
        stats["total_especialistas"] = cursor.fetchone()[0]
        
        # Total visitas
        cursor.execute("SELECT COUNT(*) FROM visitas")
        stats["total_visitas"] = cursor.fetchone()[0]
        
        # Visitas por tipo
        cursor.execute("SELECT tipo, COUNT(*) FROM visitas GROUP BY tipo")
        stats["visitas_por_tipo"] = dict(cursor.fetchall())
        
        # Informes firmados
        cursor.execute("SELECT COUNT(*) FROM informes WHERE firmado = 1")
        stats["informes_firmados"] = cursor.fetchone()[0]
        
        # Puntuación promedio
        cursor.execute("SELECT AVG(puntuacion_global) FROM informes WHERE puntuacion_global > 0")
        avg = cursor.fetchone()[0]
        stats["puntuacion_promedio"] = round(avg, 2) if avg else 0
        
        # Recomendaciones por estado
        cursor.execute("SELECT estado, COUNT(*) FROM recomendaciones GROUP BY estado")
        stats["recomendaciones_por_estado"] = dict(cursor.fetchall())
        
        # Seguimientos activos
        cursor.execute("SELECT COUNT(*) FROM seguimientos WHERE completado = 0")
        stats["seguimientos_activos"] = cursor.fetchone()[0]
        
        conn.close()
        return stats
    
    def _get_atlas_version(self) -> str:
        """Obtiene la versión actual de ATLAS"""
        try:
            # Intentar leer de archivo de versión o config
            return "3.7.0"
        except:
            return "unknown"
