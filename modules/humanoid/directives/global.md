# ATLAS NEXUS - Directivas Globales

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

## 🤖 Sistema de Directivas
- Sistema completamente implementado y operativo
- 14 endpoints API disponibles en /directives/*
- Integración automática con Neural Router
- Proyectos de ejemplo: trading_bot, rauli_erp
- Documentación completa en directives/README_DIRECTIVAS.md

## 🔑 Uso de Directivas
- Editar global: notepad directives\global.md
- Ver proyectos: Get-ChildItem directives\projects\*.md
- API REST: http://localhost:8000/directives/summary
