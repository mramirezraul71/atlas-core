"""
Generador de reportes para el sistema de Tutorías
"""

import json
import os
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Dict, Any

from .models import Visita, Informe, Especialista, Recomendacion, SeguimientoMejora


class ReportGenerator:
    """
    Genera reportes en múltiples formatos:
    - JSON
    - Markdown
    - HTML
    - Texto plano
    """
    
    def __init__(self, output_dir: str = None):
        if output_dir is None:
            base_dir = Path(__file__).parent.parent.parent.parent.parent
            self.output_dir = base_dir / "data" / "reports" / "tutorias"
        else:
            self.output_dir = Path(output_dir)
        
        self.output_dir.mkdir(parents=True, exist_ok=True)
    
    # ==================== INFORME DE VISITA ====================
    
    def generar_informe_visita(
        self,
        visita: Visita,
        formato: str = "markdown"
    ) -> str:
        """Genera un reporte completo de una visita"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"visita_{visita.id}_{timestamp}"
        
        if formato == "markdown":
            return self._informe_visita_md(visita, filename)
        elif formato == "html":
            return self._informe_visita_html(visita, filename)
        elif formato == "json":
            return self._informe_visita_json(visita, filename)
        else:
            return self._informe_visita_txt(visita, filename)
    
    def _informe_visita_md(self, visita: Visita, filename: str) -> str:
        """Genera informe en Markdown"""
        lines = []
        
        # Encabezado
        lines.append(f"# 📋 Informe de Visita: {visita.tipo.value.upper()}")
        lines.append(f"**ID:** `{visita.id}`")
        lines.append(f"**Fecha:** {visita.fecha_inicio.strftime('%Y-%m-%d %H:%M')}")
        if visita.fecha_fin:
            lines.append(f"**Duración:** {visita.duracion_minutos} minutos")
        lines.append("")
        
        # Especialista
        if visita.especialista:
            lines.append("## 👤 Especialista")
            lines.append(f"- **Nombre:** {visita.especialista.nombre}")
            lines.append(f"- **Rol:** {visita.especialista.rol}")
            lines.append(f"- **Especialidad:** {visita.especialista.especialidad}")
            lines.append(f"- **Firma Digital:** `{visita.especialista.firma_digital}`")
            lines.append("")
        
        # Contexto
        lines.append("## 📌 Contexto")
        lines.append(f"- **Motivo:** {visita.motivo}")
        lines.append(f"- **Ambiente:** {visita.ambiente}")
        lines.append(f"- **Versión ATLAS:** {visita.version_atlas}")
        lines.append("")
        
        if visita.modulos_revisados:
            lines.append("### Módulos Revisados")
            for mod in visita.modulos_revisados:
                lines.append(f"- {mod}")
            lines.append("")
        
        if visita.objetivos:
            lines.append("### Objetivos")
            for obj in visita.objetivos:
                lines.append(f"- {obj}")
            lines.append("")
        
        # Informe detallado
        if visita.informe:
            inf = visita.informe
            lines.append("---")
            lines.append(f"## 📝 {inf.titulo}")
            lines.append(f"### Resumen")
            lines.append(inf.resumen)
            lines.append("")
            
            lines.append("### Contenido")
            lines.append(inf.contenido)
            lines.append("")
            
            # Evaluaciones
            if inf.evaluaciones:
                lines.append("### 📊 Evaluaciones")
                lines.append("| Aspecto | Nivel | Puntuación | Comentario |")
                lines.append("|---------|-------|------------|------------|")
                for ev in inf.evaluaciones:
                    lines.append(f"| {ev.aspecto} | {ev.nivel.name} | {ev.puntuacion}/5 | {ev.comentario} |")
                lines.append("")
                lines.append(f"**Puntuación Global:** {inf.calcular_puntuacion_global():.2f}/5")
                lines.append("")
            
            # Recomendaciones
            if inf.recomendaciones:
                lines.append("### 💡 Recomendaciones")
                for rec in inf.recomendaciones:
                    lines.append(f"#### [{rec.prioridad.name}] {rec.titulo}")
                    lines.append(f"- **Módulo:** {rec.modulo_afectado}")
                    lines.append(f"- **Estado:** {rec.estado.name}")
                    lines.append(f"- **Descripción:** {rec.descripcion}")
                    if rec.pasos_implementacion:
                        lines.append("- **Pasos:**")
                        for i, paso in enumerate(rec.pasos_implementacion, 1):
                            lines.append(f"  {i}. {paso}")
                    lines.append("")
            
            # Objetivos cumplidos/pendientes
            if inf.objetivos_cumplidos:
                lines.append("### ✅ Objetivos Cumplidos")
                for obj in inf.objetivos_cumplidos:
                    lines.append(f"- {obj}")
                lines.append("")
            
            if inf.objetivos_pendientes:
                lines.append("### ⏳ Objetivos Pendientes")
                for obj in inf.objetivos_pendientes:
                    lines.append(f"- {obj}")
                lines.append("")
            
            # Próximos pasos
            if inf.proximos_pasos:
                lines.append("### 🔜 Próximos Pasos")
                for paso in inf.proximos_pasos:
                    lines.append(f"- {paso}")
                lines.append("")
            
            # Firma
            if inf.firmado:
                lines.append("---")
                lines.append("## ✍️ Firma Digital")
                lines.append(f"- **Firmado por:** {visita.especialista.nombre if visita.especialista else 'N/A'}")
                lines.append(f"- **Fecha de firma:** {inf.fecha_firma.strftime('%Y-%m-%d %H:%M') if inf.fecha_firma else 'N/A'}")
                lines.append(f"- **Firma:** `{inf.firma_especialista}`")
                lines.append(f"- **Hash de verificación:** `{inf.hash_verificacion}`")
        
        # Notas
        if visita.notas:
            lines.append("---")
            lines.append("## 📓 Notas Adicionales")
            lines.append(visita.notas)
        
        content = "\n".join(lines)
        
        # Guardar archivo
        filepath = self.output_dir / f"{filename}.md"
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(content)
        
        return str(filepath)
    
    def _informe_visita_html(self, visita: Visita, filename: str) -> str:
        """Genera informe en HTML"""
        html = f"""<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Informe de Visita - {visita.id}</title>
    <style>
        :root {{
            --primary: #667eea;
            --secondary: #764ba2;
            --success: #48bb78;
            --warning: #ed8936;
            --danger: #f56565;
            --bg: #1a1a2e;
            --card: #16213e;
            --text: #e2e8f0;
        }}
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, var(--bg), #0f0f1a);
            color: var(--text);
            min-height: 100vh;
            padding: 2rem;
        }}
        .container {{ max-width: 900px; margin: 0 auto; }}
        .card {{
            background: var(--card);
            border-radius: 16px;
            padding: 1.5rem;
            margin-bottom: 1.5rem;
            border: 1px solid rgba(255,255,255,0.1);
        }}
        h1 {{ 
            background: linear-gradient(90deg, var(--primary), var(--secondary));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            font-size: 2rem;
            margin-bottom: 1rem;
        }}
        h2 {{ color: var(--primary); margin: 1rem 0; border-bottom: 1px solid rgba(255,255,255,0.1); padding-bottom: 0.5rem; }}
        h3 {{ color: var(--secondary); margin: 0.8rem 0; }}
        .badge {{
            display: inline-block;
            padding: 0.25rem 0.75rem;
            border-radius: 20px;
            font-size: 0.85rem;
            font-weight: 600;
        }}
        .badge-success {{ background: var(--success); color: #000; }}
        .badge-warning {{ background: var(--warning); color: #000; }}
        .badge-danger {{ background: var(--danger); color: #fff; }}
        .badge-info {{ background: var(--primary); color: #fff; }}
        table {{ width: 100%; border-collapse: collapse; margin: 1rem 0; }}
        th, td {{ padding: 0.75rem; text-align: left; border-bottom: 1px solid rgba(255,255,255,0.1); }}
        th {{ color: var(--primary); }}
        .signature {{
            background: linear-gradient(135deg, rgba(102,126,234,0.2), rgba(118,75,162,0.2));
            border-radius: 12px;
            padding: 1rem;
            margin-top: 1rem;
        }}
        .meta {{ color: #888; font-size: 0.9rem; }}
        code {{ background: rgba(255,255,255,0.1); padding: 0.2rem 0.5rem; border-radius: 4px; font-family: monospace; }}
        ul {{ margin: 0.5rem 0; padding-left: 1.5rem; }}
        li {{ margin: 0.3rem 0; }}
        .score {{ font-size: 2rem; font-weight: bold; color: var(--success); }}
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <h1>📋 Informe de Visita: {visita.tipo.value.upper()}</h1>
            <p class="meta">ID: <code>{visita.id}</code> | Fecha: {visita.fecha_inicio.strftime('%Y-%m-%d %H:%M')}</p>
            {f'<p class="meta">Duración: {visita.duracion_minutos} minutos</p>' if visita.fecha_fin else ''}
        </div>
"""
        
        # Especialista
        if visita.especialista:
            esp = visita.especialista
            html += f"""
        <div class="card">
            <h2>👤 Especialista</h2>
            <p><strong>Nombre:</strong> {esp.nombre}</p>
            <p><strong>Rol:</strong> {esp.rol}</p>
            <p><strong>Especialidad:</strong> {esp.especialidad}</p>
            <p><strong>Firma Digital:</strong> <code>{esp.firma_digital}</code></p>
        </div>
"""
        
        # Contexto
        html += f"""
        <div class="card">
            <h2>📌 Contexto</h2>
            <p><strong>Motivo:</strong> {visita.motivo}</p>
            <p><strong>Ambiente:</strong> <span class="badge badge-info">{visita.ambiente}</span></p>
            <p><strong>Versión ATLAS:</strong> <code>{visita.version_atlas}</code></p>
"""
        if visita.modulos_revisados:
            html += "<h3>Módulos Revisados</h3><ul>"
            for mod in visita.modulos_revisados:
                html += f"<li>{mod}</li>"
            html += "</ul>"
        
        if visita.objetivos:
            html += "<h3>Objetivos</h3><ul>"
            for obj in visita.objetivos:
                html += f"<li>{obj}</li>"
            html += "</ul>"
        
        html += "</div>"
        
        # Informe detallado
        if visita.informe:
            inf = visita.informe
            html += f"""
        <div class="card">
            <h2>📝 {inf.titulo}</h2>
            <h3>Resumen</h3>
            <p>{inf.resumen}</p>
            <h3>Contenido</h3>
            <p>{inf.contenido.replace(chr(10), '<br>')}</p>
"""
            
            # Evaluaciones
            if inf.evaluaciones:
                html += """
            <h3>📊 Evaluaciones</h3>
            <table>
                <tr><th>Aspecto</th><th>Nivel</th><th>Puntuación</th><th>Comentario</th></tr>
"""
                for ev in inf.evaluaciones:
                    badge_class = "badge-success" if ev.puntuacion >= 4 else "badge-warning" if ev.puntuacion >= 3 else "badge-danger"
                    html += f"""
                <tr>
                    <td>{ev.aspecto}</td>
                    <td><span class="badge {badge_class}">{ev.nivel.name}</span></td>
                    <td>{ev.puntuacion}/5</td>
                    <td>{ev.comentario}</td>
                </tr>
"""
                html += f"""
            </table>
            <p>Puntuación Global: <span class="score">{inf.calcular_puntuacion_global():.1f}/5</span></p>
"""
            
            # Recomendaciones
            if inf.recomendaciones:
                html += "<h3>💡 Recomendaciones</h3>"
                for rec in inf.recomendaciones:
                    priority_class = "badge-danger" if rec.prioridad.value <= 2 else "badge-warning" if rec.prioridad.value == 3 else "badge-info"
                    html += f"""
            <div style="margin: 1rem 0; padding: 1rem; background: rgba(0,0,0,0.2); border-radius: 8px;">
                <h4><span class="badge {priority_class}">{rec.prioridad.name}</span> {rec.titulo}</h4>
                <p><strong>Módulo:</strong> {rec.modulo_afectado}</p>
                <p>{rec.descripcion}</p>
"""
                    if rec.pasos_implementacion:
                        html += "<p><strong>Pasos:</strong></p><ol>"
                        for paso in rec.pasos_implementacion:
                            html += f"<li>{paso}</li>"
                        html += "</ol>"
                    html += "</div>"
            
            # Firma
            if inf.firmado:
                html += f"""
            <div class="signature">
                <h3>✍️ Firma Digital</h3>
                <p><strong>Firmado por:</strong> {visita.especialista.nombre if visita.especialista else 'N/A'}</p>
                <p><strong>Fecha:</strong> {inf.fecha_firma.strftime('%Y-%m-%d %H:%M') if inf.fecha_firma else 'N/A'}</p>
                <p><strong>Firma:</strong> <code>{inf.firma_especialista}</code></p>
                <p><strong>Hash:</strong> <code>{inf.hash_verificacion}</code></p>
            </div>
"""
            
            html += "</div>"
        
        html += """
    </div>
</body>
</html>
"""
        
        # Guardar archivo
        filepath = self.output_dir / f"{filename}.html"
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(html)
        
        return str(filepath)
    
    def _informe_visita_json(self, visita: Visita, filename: str) -> str:
        """Genera informe en JSON"""
        filepath = self.output_dir / f"{filename}.json"
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(visita.to_dict(), f, indent=2, ensure_ascii=False)
        return str(filepath)
    
    def _informe_visita_txt(self, visita: Visita, filename: str) -> str:
        """Genera informe en texto plano"""
        lines = []
        lines.append("=" * 60)
        lines.append(f"INFORME DE VISITA: {visita.tipo.value.upper()}")
        lines.append("=" * 60)
        lines.append(f"ID: {visita.id}")
        lines.append(f"Fecha: {visita.fecha_inicio.strftime('%Y-%m-%d %H:%M')}")
        if visita.fecha_fin:
            lines.append(f"Duración: {visita.duracion_minutos} minutos")
        lines.append("")
        
        if visita.especialista:
            lines.append("-" * 40)
            lines.append("ESPECIALISTA")
            lines.append("-" * 40)
            lines.append(f"Nombre: {visita.especialista.nombre}")
            lines.append(f"Rol: {visita.especialista.rol}")
            lines.append(f"Especialidad: {visita.especialista.especialidad}")
            lines.append(f"Firma: {visita.especialista.firma_digital}")
            lines.append("")
        
        if visita.informe:
            inf = visita.informe
            lines.append("-" * 40)
            lines.append(f"INFORME: {inf.titulo}")
            lines.append("-" * 40)
            lines.append(f"Resumen: {inf.resumen}")
            lines.append("")
            lines.append("Contenido:")
            lines.append(inf.contenido)
            lines.append("")
            
            if inf.evaluaciones:
                lines.append("EVALUACIONES:")
                for ev in inf.evaluaciones:
                    lines.append(f"  - {ev.aspecto}: {ev.puntuacion}/5 ({ev.nivel.name})")
                lines.append(f"  PUNTUACIÓN GLOBAL: {inf.calcular_puntuacion_global():.2f}/5")
                lines.append("")
            
            if inf.firmado:
                lines.append("-" * 40)
                lines.append("FIRMA DIGITAL")
                lines.append(f"Firma: {inf.firma_especialista}")
                lines.append(f"Hash: {inf.hash_verificacion}")
                lines.append(f"Fecha: {inf.fecha_firma.strftime('%Y-%m-%d %H:%M') if inf.fecha_firma else 'N/A'}")
        
        lines.append("")
        lines.append("=" * 60)
        lines.append("FIN DEL INFORME")
        lines.append("=" * 60)
        
        content = "\n".join(lines)
        filepath = self.output_dir / f"{filename}.txt"
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(content)
        
        return str(filepath)
    
    # ==================== RESUMEN DE RECOMENDACIONES ====================
    
    def generar_resumen_recomendaciones(
        self,
        recomendaciones: List[Recomendacion],
        formato: str = "markdown"
    ) -> str:
        """Genera un resumen de recomendaciones pendientes"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"recomendaciones_{timestamp}"
        
        if formato == "markdown":
            lines = []
            lines.append("# 📋 Resumen de Recomendaciones")
            lines.append(f"*Generado: {datetime.now().strftime('%Y-%m-%d %H:%M')}*")
            lines.append("")
            
            # Agrupar por prioridad
            by_priority = {}
            for rec in recomendaciones:
                p = rec.prioridad.name
                if p not in by_priority:
                    by_priority[p] = []
                by_priority[p].append(rec)
            
            for priority in ["CRITICA", "ALTA", "MEDIA", "BAJA", "OPCIONAL"]:
                if priority in by_priority:
                    emoji = "🔴" if priority == "CRITICA" else "🟠" if priority == "ALTA" else "🟡" if priority == "MEDIA" else "🟢"
                    lines.append(f"## {emoji} {priority}")
                    for rec in by_priority[priority]:
                        lines.append(f"### {rec.titulo}")
                        lines.append(f"- **Módulo:** {rec.modulo_afectado}")
                        lines.append(f"- **Estado:** {rec.estado.name}")
                        lines.append(f"- {rec.descripcion}")
                        lines.append("")
            
            content = "\n".join(lines)
            filepath = self.output_dir / f"{filename}.md"
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(content)
            return str(filepath)
        
        return ""
    
    # ==================== DASHBOARD DE ESTADO ====================
    
    def generar_dashboard_estado(
        self,
        estadisticas: Dict[str, Any],
        seguimientos: List[SeguimientoMejora]
    ) -> str:
        """Genera un dashboard HTML con el estado actual"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"dashboard_tutorias_{timestamp}"
        
        html = f"""<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Dashboard de Tutorías - ATLAS</title>
    <style>
        :root {{
            --primary: #667eea;
            --secondary: #764ba2;
            --success: #48bb78;
            --warning: #ed8936;
            --danger: #f56565;
            --bg: #1a1a2e;
            --card: #16213e;
            --text: #e2e8f0;
        }}
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, var(--bg), #0f0f1a);
            color: var(--text);
            min-height: 100vh;
            padding: 2rem;
        }}
        .container {{ max-width: 1200px; margin: 0 auto; }}
        h1 {{
            background: linear-gradient(90deg, var(--primary), var(--secondary));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            font-size: 2.5rem;
            text-align: center;
            margin-bottom: 2rem;
        }}
        .grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 1.5rem;
            margin-bottom: 2rem;
        }}
        .stat-card {{
            background: var(--card);
            border-radius: 16px;
            padding: 1.5rem;
            text-align: center;
            border: 1px solid rgba(255,255,255,0.1);
        }}
        .stat-value {{
            font-size: 3rem;
            font-weight: bold;
            background: linear-gradient(90deg, var(--primary), var(--secondary));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }}
        .stat-label {{ color: #888; margin-top: 0.5rem; }}
        .card {{
            background: var(--card);
            border-radius: 16px;
            padding: 1.5rem;
            margin-bottom: 1.5rem;
            border: 1px solid rgba(255,255,255,0.1);
        }}
        h2 {{ color: var(--primary); margin-bottom: 1rem; }}
        .progress {{
            background: rgba(255,255,255,0.1);
            border-radius: 10px;
            height: 20px;
            overflow: hidden;
            margin: 0.5rem 0;
        }}
        .progress-bar {{
            height: 100%;
            background: linear-gradient(90deg, var(--primary), var(--secondary));
            border-radius: 10px;
            transition: width 0.3s;
        }}
        .seguimiento {{
            padding: 1rem;
            background: rgba(0,0,0,0.2);
            border-radius: 8px;
            margin: 0.5rem 0;
        }}
        .meta {{ color: #888; font-size: 0.9rem; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>📊 Dashboard de Tutorías</h1>
        <p class="meta" style="text-align: center; margin-bottom: 2rem;">
            Actualizado: {datetime.now().strftime('%Y-%m-%d %H:%M')}
        </p>
        
        <div class="grid">
            <div class="stat-card">
                <div class="stat-value">{estadisticas.get('total_especialistas', 0)}</div>
                <div class="stat-label">Especialistas</div>
            </div>
            <div class="stat-card">
                <div class="stat-value">{estadisticas.get('total_visitas', 0)}</div>
                <div class="stat-label">Visitas Totales</div>
            </div>
            <div class="stat-card">
                <div class="stat-value">{estadisticas.get('informes_firmados', 0)}</div>
                <div class="stat-label">Informes Firmados</div>
            </div>
            <div class="stat-card">
                <div class="stat-value">{estadisticas.get('puntuacion_promedio', 0)}</div>
                <div class="stat-label">Puntuación Promedio</div>
            </div>
            <div class="stat-card">
                <div class="stat-value">{estadisticas.get('seguimientos_activos', 0)}</div>
                <div class="stat-label">Seguimientos Activos</div>
            </div>
        </div>
        
        <div class="card">
            <h2>📈 Seguimientos en Progreso</h2>
"""
        
        for seg in seguimientos[:10]:  # Mostrar los 10 más recientes
            html += f"""
            <div class="seguimiento">
                <strong>{seg.titulo}</strong>
                <div class="progress">
                    <div class="progress-bar" style="width: {seg.porcentaje_avance}%"></div>
                </div>
                <p class="meta">{seg.porcentaje_avance}% completado | Responsable: {seg.responsable}</p>
            </div>
"""
        
        html += """
        </div>
    </div>
</body>
</html>
"""
        
        filepath = self.output_dir / f"{filename}.html"
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(html)
        
        return str(filepath)
