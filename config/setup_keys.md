# Configuración de API Keys para Atlas

## OpenAI API Key (Requerido para Mem0)

1. Obtén tu API key en: https://platform.openai.com/account/api-keys
2. Copia la key (empieza con sk-)
3. Edita este archivo y reemplaza TU_API_KEY_AQUI

```bash
# Opción 1: Variable de entorno temporal
$env:OPENAI_API_KEY = "sk-TU_API_KEY_AQUI"

# Opción 2: Agregar a tu perfil de PowerShell
echo '$env:OPENAI_API_KEY = "sk-TU_API_KEY_AQUI"' >> $PROFILE

# Opción 3: Crear archivo .env (no funciona si está en .gitignore)
```

## Composio API Key (Opcional)

1. Regístrate en: https://composio.dev
2. Obtén tu API key
3. Configura:

```bash
$env:COMPOSIO_API_KEY = "tu-composio-api-key"
```

## Verificación

Ejecuta para verificar:

```bash
# Verificar OpenAI key
python -c "import os; print('OpenAI Key:', 'OK' if os.getenv('OPENAI_API_KEY','').startswith('sk-') else 'NOT SET')"

# Verificar servicios
python config/atlas_tools_config.py
```

## Servicios Activados

✅ **Docker Desktop**: Corriendo
✅ **Home Assistant**: http://localhost:8123
✅ **Appsmith**: http://localhost
✅ **Ollama**: http://localhost:11434 (con modelos descargados)

## Siguientes Pasos

1. **Configura tu OpenAI API key** arriba
2. **Ejecuta prueba de integración**:
   ```bash
   $env:OPENAI_API_KEY = "sk-TU_REAL_KEY"
   python workspace_prime/tools_integration.py
   ```
3. **Usa desde el agente workspace**:
   ```python
   from tools_integration import tools
   tools.mem0_store_memory("Cliente Juan prefiere EUR/USD", ["clientes"])
   ```
