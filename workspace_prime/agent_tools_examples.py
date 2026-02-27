"""
Ejemplos de uso de las herramientas integradas desde el agente workspace
"""

from tools_integration import tools
import json

def demo_mem0_memory():
    """Demostración de Mem0 para memoria de clientes y hábitos"""
    print("=== MEM0 - Memoria de Clientes y Hábitos ===")
    
    # Almacenar información de cliente
    result = tools.mem0_store_memory(
        "Cliente Raúl Martínez: prefiere trading automatizado, usa MT4, le gustan los pares EUR/USD y GBP/JPY",
        ["clientes", "raul_martinez", "trading", "preferencias"]
    )
    print(f"✅ Memoria cliente: {result}")
    
    # Almacenar hábitos
    result = tools.mem0_store_memory(
        "Hábito: Revisar dashboard de trading cada 30 minutos durante sesión europea",
        ["habitos", "trading", "dashboard", "rutina"]
    )
    print(f"✅ Memoria hábito: {result}")
    
    # Buscar memorias
    result = tools.mem0_search_memories("trading")
    print(f"🔍 Búsqueda 'trading': {result}")

def demo_composio_connectivity():
    """Demostración de Composio para apps externas"""
    print("\n=== COMPOSIO - Conectividad Apps Externas ===")
    
    # Conectar Google Sheets
    result = tools.composio_connect_app("google_sheets", {
        "api_key": "mock-api-key",
        "spreadsheet_id": "trading_dashboard_sheet"
    })
    print(f"📊 Conexión Google Sheets: {result}")
    
    # Ejecutar acción en Google Sheets
    result = tools.composio_execute_action("google_sheets", "update_cell", {
        "range": "A1",
        "value": "Nuevo trade ejecutado"
    })
    print(f"📝 Actualizar celda: {result}")

def demo_ollama_ai():
    """Demostración de Ollama para motor de IA privado"""
    print("\n=== OLLAMA - Motor IA Privado ===")
    
    # Listar modelos
    result = tools.ollama_list_models()
    print(f"🤖 Modelos disponibles: {result}")
    
    # Generar respuesta
    result = tools.ollama_generate(
        "Analiza este patrón de trading: breakout en EUR/USD con volumen alto",
        model="llama2"
    )
    print(f"🧠 Generación IA: {result}")

def demo_homeassistant_iot():
    """Demostración de Home Assistant para IoT/Hardware"""
    print("\n=== HOME ASSISTANT - IoT/Hardware ===")
    
    # Obtener dispositivos
    result = tools.home_assistant_get_devices()
    print(f"🏠 Dispositivos: {result}")
    
    # Controlar dispositivo (ejemplo: ATLAS_NEXUS)
    result = tools.home_assistant_control_device("sensor.atlas_nexus_status", "turn_on")
    print(f"🎛️ Control dispositivo: {result}")

def demo_appsmith_interface():
    """Demostración de Appsmith para interfaz visual"""
    print("\n=== APPSMITH - Interfaz Visual ===")
    
    # Crear dashboard de trading
    dashboard_config = {
        "widgets": [
            {"type": "chart", "title": "Ganancias diarias", "data_source": "trading_db"},
            {"type": "table", "title": "Trades activos", "data_source": "trades_table"},
            {"type": "metric", "title": "ROI total", "value": "15.3%"}
        ],
        "layout": "grid",
        "theme": "dark"
    }
    
    result = tools.appsmith_create_dashboard("Atlas Trading Dashboard", dashboard_config)
    print(f"📈 Dashboard creado: {result}")
    
    # Listar dashboards
    result = tools.appsmith_get_dashboards()
    print(f"📊 Dashboards: {result}")

def demo_complete_workflow():
    """Demostración completa de workflow integrado"""
    print("\n=== WORKFLOW INTEGRADO COMPLETO ===")
    
    # 1. Recordar información del cliente (Mem0)
    tools.mem0_store_memory(
        "Cliente solicita análisis de patrón de velas en timeframe H4",
        ["clientes", "analisis", "velas", "H4"]
    )
    
    # 2. Generar análisis con IA local (Ollama)
    analysis = tools.ollama_generate(
        "Analiza patrón de velas japonesas para detectar posibles reversiones",
        model="llama2"
    )
    
    # 3. Registrar en Google Sheets (Composio)
    tools.composio_execute_action("google_sheets", "append_row", {
        "worksheet": "analisis_tecnicos",
        "data": ["Análisis velas", analysis.get("response", ""), "H4"]
    })
    
    # 4. Actualizar dashboard (Appsmith)
    tools.appsmith_create_dashboard("Análisis Técnico", {
        "widgets": [{"type": "text", "content": analysis.get("response", "")}]
    })
    
    # 5. Notificar via IoT (Home Assistant)
    tools.home_assistant_control_device("notification.atlas_alert", "send_notification")
    
    print("✅ Workflow completado")
    
    # Mostrar estado final
    status = tools.get_all_status()
    print(f"\n📊 Estado final: {json.dumps(status, indent=2)}")

if __name__ == "__main__":
    print("🚀 Demostración de Herramientas Integradas para Agente Workspace")
    print("=" * 60)
    
    # Ejecutar todas las demos
    demo_mem0_memory()
    demo_composio_connectivity()
    demo_ollama_ai()
    demo_homeassistant_iot()
    demo_appsmith_interface()
    demo_complete_workflow()
    
    print("\n🎯 Todas las herramientas están listas para usar desde el agente workspace!")
