#!/usr/bin/env python3
"""
ATLAS NEXUS - Directives Manager
Sistema de gestión de directivas globales y por proyecto
"""

import os
import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from datetime import datetime
import hashlib

logger = logging.getLogger(__name__)

class DirectivesManager:
    """Gestor principal del sistema de directivas"""
    
    def __init__(self, base_path: str = None):
        """Inicializar el gestor de directivas"""
        if base_path is None:
            # Ruta por defecto dentro de ATLAS NEXUS
            self.base_path = Path(__file__).parent
        else:
            self.base_path = Path(base_path)
            
        self.global_file = self.base_path / "global.md"
        self.projects_dir = self.base_path / "projects"
        self.metadata_file = self.base_path / "metadata.json"
        
        # Asegurar que las carpetas existan
        self._initialize_structure()
        
    def _initialize_structure(self):
        """Crear estructura de directorios si no existe"""
        try:
            self.projects_dir.mkdir(exist_ok=True)
            
            # Crear metadata.json si no existe
            if not self.metadata_file.exists():
                self._create_default_metadata()
                
            logger.info(f"Estructura de directivas inicializada en: {self.base_path}")
            
        except Exception as e:
            logger.error(f"Error inicializando estructura: {e}")
            
    def _create_default_metadata(self):
        """Crear archivo de metadata por defecto"""
        default_metadata = {
            "version": "1.0.0",
            "created": datetime.now().isoformat(),
            "last_updated": datetime.now().isoformat(),
            "global_enabled": True,
            "projects": {},
            "stats": {
                "total_projects": 0,
                "total_directives": 0,
                "last_backup": None
            }
        }
        
        with open(self.metadata_file, 'w', encoding='utf-8') as f:
            json.dump(default_metadata, f, indent=2, ensure_ascii=False)
            
    def get_global_directives(self) -> str:
        """Obtener directivas globales"""
        try:
            if self.global_file.exists():
                with open(self.global_file, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                    return content if content else ""
            return ""
        except Exception as e:
            logger.error(f"Error leyendo directivas globales: {e}")
            return ""
            
    def set_global_directives(self, content: str) -> bool:
        """Establecer directivas globales"""
        try:
            with open(self.global_file, 'w', encoding='utf-8') as f:
                f.write(content)
                
            self._update_metadata()
            logger.info("Directivas globales actualizadas")
            return True
            
        except Exception as e:
            logger.error(f"Error estableciendo directivas globales: {e}")
            return False
            
    def append_global_directives(self, content: str) -> bool:
        """Agregar contenido a directivas globales"""
        try:
            current = self.get_global_directives()
            new_content = current + "\n\n" + content if current else content
            return self.set_global_directives(new_content)
            
        except Exception as e:
            logger.error(f"Error agregando a directivas globales: {e}")
            return False
            
    def get_project_directives(self, project_name: str) -> str:
        """Obtener directivas de un proyecto"""
        try:
            project_file = self.projects_dir / f"{project_name}.md"
            if project_file.exists():
                with open(project_file, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                    return content if content else ""
            return ""
        except Exception as e:
            logger.error(f"Error leyendo directivas del proyecto {project_name}: {e}")
            return ""
            
    def set_project_directives(self, project_name: str, content: str) -> bool:
        """Establecer directivas de un proyecto"""
        try:
            project_file = self.projects_dir / f"{project_name}.md"
            with open(project_file, 'w', encoding='utf-8') as f:
                f.write(content)
                
            self._update_metadata()
            logger.info(f"Directivas del proyecto '{project_name}' actualizadas")
            return True
            
        except Exception as e:
            logger.error(f"Error estableciendo directivas del proyecto {project_name}: {e}")
            return False
            
    def delete_project(self, project_name: str) -> bool:
        """Eliminar un proyecto"""
        try:
            project_file = self.projects_dir / f"{project_name}.md"
            if project_file.exists():
                project_file.unlink()
                self._update_metadata()
                logger.info(f"Proyecto '{project_name}' eliminado")
                return True
            return False
            
        except Exception as e:
            logger.error(f"Error eliminando proyecto {project_name}: {e}")
            return False
            
    def list_projects(self) -> List[str]:
        """Listar todos los proyectos"""
        try:
            projects = []
            for file_path in self.projects_dir.glob("*.md"):
                projects.append(file_path.stem)
            return sorted(projects)
            
        except Exception as e:
            logger.error(f"Error listando proyectos: {e}")
            return []
            
    def get_active_directives(self, project_name: str = None) -> str:
        """Obtener directivas activas (globales + de proyecto si se especifica)"""
        try:
            directives = []
            
            # Directivas globales
            global_directives = self.get_global_directives()
            if global_directives:
                directives.append(global_directives)
                
            # Directivas del proyecto
            if project_name:
                project_directives = self.get_project_directives(project_name)
                if project_directives:
                    directives.append(project_directives)
                    
            return "\n\n---\n\n".join(directives) if directives else ""
            
        except Exception as e:
            logger.error(f"Error obteniendo directivas activas: {e}")
            return ""
            
    def get_summary(self) -> Dict:
        """Obtener resumen completo del sistema"""
        try:
            metadata = self._load_metadata()
            projects = self.list_projects()
            
            summary = {
                "global_enabled": metadata.get("global_enabled", True),
                "global_directives": self.get_global_directives(),
                "projects": {},
                "stats": {
                    "total_projects": len(projects),
                    "global_size": len(self.get_global_directives()),
                    "last_updated": metadata.get("last_updated")
                }
            }
            
            # Información de cada proyecto
            for project in projects:
                project_content = self.get_project_directives(project)
                summary["projects"][project] = {
                    "enabled": True,  # TODO: Implementar enable/disable por proyecto
                    "size": len(project_content),
                    "preview": project_content[:200] + "..." if len(project_content) > 200 else project_content
                }
                
            return summary
            
        except Exception as e:
            logger.error(f"Error obteniendo resumen: {e}")
            return {}
            
    def create_project_template(self, project_name: str, description: str = "") -> bool:
        """Crear un proyecto desde template"""
        try:
            template = f"""# {project_name} - Directivas

## 📋 Descripción
{description or f"Proyecto {project_name}"}

## 🛠️ Stack Tecnológico
- [Tecnología 1]
- [Tecnología 2]
- [Tecnología 3]

## 📋 Reglas de Código
- [Regla 1]
- [Regla 2]
- [Regla 3]

## 🎨 Convenciones
- [Convención 1]
- [Convención 2]

## 📝 Notas Importantes
- [Nota 1]
- [Nota 2]

---

**Creado:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
**Por:** ATLAS NEXUS
"""
            
            return self.set_project_directives(project_name, template)
            
        except Exception as e:
            logger.error(f"Error creando template para {project_name}: {e}")
            return False
            
    def toggle_global(self, enabled: bool) -> bool:
        """Activar/desactivar directivas globales"""
        try:
            metadata = self._load_metadata()
            metadata["global_enabled"] = enabled
            self._save_metadata(metadata)
            
            status = "activadas" if enabled else "desactivadas"
            logger.info(f"Directivas globales {status}")
            return True
            
        except Exception as e:
            logger.error(f"Error cambiando estado de directivas globales: {e}")
            return False
            
    def toggle_project(self, project_name: str, enabled: bool) -> bool:
        """Activar/desactivar directivas de proyecto"""
        try:
            metadata = self._load_metadata()
            
            if "projects" not in metadata:
                metadata["projects"] = {}
                
            if project_name not in metadata["projects"]:
                metadata["projects"][project_name] = {}
                
            metadata["projects"][project_name]["enabled"] = enabled
            metadata["projects"][project_name]["last_updated"] = datetime.now().isoformat()
            
            self._save_metadata(metadata)
            
            status = "activadas" if enabled else "desactivadas"
            logger.info(f"Directivas del proyecto '{project_name}' {status}")
            return True
            
        except Exception as e:
            logger.error(f"Error cambiando estado del proyecto {project_name}: {e}")
            return False
            
    def _load_metadata(self) -> Dict:
        """Cargar metadata desde archivo"""
        try:
            if self.metadata_file.exists():
                with open(self.metadata_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            return {}
        except Exception as e:
            logger.error(f"Error cargando metadata: {e}")
            return {}
            
    def _save_metadata(self, metadata: Dict):
        """Guardar metadata a archivo"""
        try:
            metadata["last_updated"] = datetime.now().isoformat()
            with open(self.metadata_file, 'w', encoding='utf-8') as f:
                json.dump(metadata, f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.error(f"Error guardando metadata: {e}")
            
    def _update_metadata(self):
        """Actualizar metadata después de cambios"""
        try:
            metadata = self._load_metadata()
            projects = self.list_projects()
            
            metadata["last_updated"] = datetime.now().isoformat()
            metadata["stats"]["total_projects"] = len(projects)
            metadata["stats"]["total_directives"] = len(projects) + (1 if self.global_file.exists() else 0)
            
            # Actualizar información de proyectos
            if "projects" not in metadata:
                metadata["projects"] = {}
                
            for project in projects:
                if project not in metadata["projects"]:
                    metadata["projects"][project] = {
                        "created": datetime.now().isoformat(),
                        "enabled": True
                    }
                    
            self._save_metadata(metadata)
            
        except Exception as e:
            logger.error(f"Error actualizando metadata: {e}")
            
    def create_default_directives(self) -> bool:
        """Crear directivas por defecto"""
        try:
            # Directivas globales por defecto
            default_global = """# ATLAS NEXUS - Directivas Globales

## 🎯 Instrucciones Permanentes

### 1. Lenguaje y Comunicación
- Español por defecto, inglés para documentación técnica
- Tono: Profesional pero amigable
- Sé conciso y directo

### 2. Estándares de Código
- Python: PEP 8, type hints, docstrings estilo Google
- JavaScript: ES6+, async/await, comentarios JSDoc
- Siempre incluir comentarios explicativos

### 3. Estructura de Proyectos
```
proyecto/
├── src/          # Código fuente
├── tests/        # Tests unitarios
├── docs/         # Documentación
├── config/       # Configuración
└── README.md     # Documentación principal
```

### 4. Credenciales y Seguridad
- Leer SIEMPRE desde C:\dev\credenciales.txt
- Nunca hardcodear API keys o contraseñas
- Validar todos los inputs de usuario
- Usar prepared statements en SQL

### 5. Testing
- Crea tests para código crítico
- Usa pytest para Python, Jest para JavaScript
- Coverage mínimo: 80%

### 6. Git y Versionado
- Commits en inglés, formato: "type(scope): message"
- Branches: feature/, bugfix/, hotfix/
- Documentar cambios importantes

### 7. Documentación
- README.md completo y claro
- Comentarios inline para lógica compleja
- API docs con ejemplos de uso

---

**Creado:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
**Por:** ATLAS NEXUS
"""
            
            return self.set_global_directives(default_global)
            
        except Exception as e:
            logger.error(f"Error creando directivas por defecto: {e}")
            return False

# Instancia global del gestor
directives_manager = DirectivesManager()

# Funciones de conveniencia para importación fácil
def get_directives_manager() -> DirectivesManager:
    """Obtener instancia global del DirectivesManager"""
    return directives_manager

def get_global_directives() -> str:
    return directives_manager.get_global_directives()

def get_project_directives(project_name: str) -> str:
    return directives_manager.get_project_directives(project_name)

def get_active_directives(project_name: str = None) -> str:
    return directives_manager.get_active_directives(project_name)

def list_projects() -> List[str]:
    return directives_manager.list_projects()

def get_summary() -> Dict:
    return directives_manager.get_summary()
