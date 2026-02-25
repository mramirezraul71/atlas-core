"""
Ejemplos de Workflows de Automatización para Atlas
Usando las 5 herramientas gratuitas: Ollama, n8n, Home Assistant, Appsmith, Composio
"""

import json

from tools_integration import tools


def workflow_trading_automation():
    """Workflow completo de trading automatizado"""
    print("🚀 Workflow: Trading Automatizado")
    print("=" * 50)

    # 1. Análisis con IA local (Ollama)
    print("\n1️⃣ Análisis de mercado con IA local...")
    analysis = tools.ollama_generate(
        "Analiza EUR/USD para posible entrada larga, considera soporte y resistencia",
        model="llama3.2:3b",
    )
    print(f"📊 Análisis IA: {analysis['response'][:200]}...")

    # 2. Crear dashboard en Appsmith
    print("\n2️⃣ Creando dashboard de trading...")
    dashboard_config = {
        "widgets": [
            {"type": "chart", "title": "Análisis EUR/USD", "data_source": "analysis"},
            {
                "type": "metric",
                "title": "Señal",
                "value": "LARGA"
                if "larga" in analysis["response"].lower()
                else "CORTA",
            },
            {"type": "table", "title": "Niveles Clave", "data_source": "levels"},
        ]
    }

    dashboard = tools.appsmith_create_dashboard(
        "Atlas Trading Analysis", dashboard_config
    )
    print(f"📈 Dashboard: {dashboard['dashboard']['url']}")

    # 3. Notificar via Home Assistant
    print("\n3️⃣ Enviando notificación a ATLAS_NEXUS...")
    notification = tools.home_assistant_control_device(
        "notification.atlas_trading",
        f"Señal trading detectada: {analysis['response'][:100]}",
    )
    print(f"🏠 Notificación IoT: {notification}")

    # 4. Crear workflow en n8n
    print("\n4️⃣ Configurando workflow de automatización...")
    workflow = tools.n8n_create_workflow(
        "Trading Automation",
        [
            {
                "parameters": {"content": analysis["response"]},
                "id": "analysis-node",
                "name": "Análisis IA",
                "type": "n8n-nodes-base.httpRequest",
                "position": [240, 300],
            },
            {
                "parameters": {"message": "Nueva señal de trading generada"},
                "id": "notification-node",
                "name": "Notificación",
                "type": "n8n-nodes-base.gmail",
                "position": [440, 300],
            },
        ],
    )
    print(f"⚙️ Workflow n8n: {workflow}")

    return {
        "analysis": analysis["response"],
        "dashboard": dashboard["dashboard"]["url"],
        "notification": notification,
        "workflow": workflow,
    }


def workflow_cliente_inteligencia():
    """Workflow de inteligencia de clientes"""
    print("\n🧠 Workflow: Inteligencia de Clientes")
    print("=" * 50)

    # 1. Análisis de comportamiento con IA
    print("\n1️⃣ Analizando patrón de cliente...")
    client_analysis = tools.ollama_generate(
        "Analiza patrón de cliente que opera EUR/USD y GBP/JPY, sugiere estrategias personalizadas",
        model="deepseek-coder:6.7b",
    )

    # 2. Configurar conexión con Gmail (Composio)
    print("\n2️⃣ Configurando Gmail para seguimiento...")
    gmail_connection = tools.composio_connect_app(
        "gmail", {"email": "trading@atlas.com", "scope": ["send", "read"]}
    )

    # 3. Crear dashboard de cliente
    print("\n3️⃣ Dashboard personalizado del cliente...")
    client_dashboard = tools.appsmith_create_dashboard(
        "Cliente Intelligence",
        {
            "widgets": [
                {
                    "type": "text",
                    "content": f"Análisis: {client_analysis['response'][:200]}",
                },
                {
                    "type": "metric",
                    "title": "Preferencia",
                    "value": "EUR/USD + GBP/JPY",
                },
                {
                    "type": "chart",
                    "title": "Rendimiento",
                    "data_source": "client_trades",
                },
            ]
        },
    )

    # 4. Workflow de seguimiento en n8n
    print("\n4️⃣ Workflow de seguimiento automático...")
    followup_workflow = tools.n8n_create_workflow(
        "Cliente Followup",
        [
            {
                "parameters": {"schedule": "0 9 * * 1-5"},  # Lunes a Viernes 9am
                "id": "schedule-node",
                "name": "Programador",
                "type": "n8n-nodes-base.cron",
                "position": [240, 300],
            },
            {
                "parameters": {"template": "client_update"},
                "id": "email-node",
                "name": "Email Cliente",
                "type": "n8n-nodes-base.gmail",
                "position": [440, 300],
            },
        ],
    )

    return {
        "analysis": client_analysis["response"],
        "gmail": gmail_connection,
        "dashboard": client_dashboard["dashboard"]["url"],
        "workflow": followup_workflow,
    }


def workflow_monitoring_iot():
    """Workflow de monitoreo IoT con ATLAS_NEXUS"""
    print("\n🏠 Workflow: Monitoreo ATLAS_NEXUS")
    print("=" * 50)

    # 1. Verificar dispositivos IoT
    print("\n1️⃣ Verificando dispositivos ATLAS_NEXUS...")
    devices = tools.home_assistant_get_devices()

    # 2. Análisis de estado con IA
    print("\n2️⃣ Analizando estado del sistema...")
    status_analysis = tools.ollama_generate(
        f"Analiza estado de dispositivos IoT: {len(devices.get('devices', []))} activos. Sugiere optimizaciones.",
        model="llama3.1:latest",
    )

    if not status_analysis.get("ok"):
        status_text = "Análisis no disponible temporalmente"
    else:
        status_text = status_analysis.get("response", "Análisis no disponible")

    # 3. Dashboard de monitoreo
    print("\n3️⃣ Dashboard de monitoreo IoT...")
    monitoring_dashboard = tools.appsmith_create_dashboard(
        "ATLAS NEXUS Monitor",
        {
            "widgets": [
                {
                    "type": "metric",
                    "title": "Dispositivos Activos",
                    "value": len(devices.get("devices", [])),
                },
                {"type": "text", "content": f"Análisis: {status_text[:200]}"},
                {
                    "type": "table",
                    "title": "Estado Dispositivos",
                    "data_source": "iot_devices",
                },
            ]
        },
    )

    # 4. Workflow de alertas
    print("\n4️⃣ Configurando sistema de alertas...")
    alert_workflow = tools.n8n_create_workflow(
        "IoT Alerts",
        [
            {
                "parameters": {"trigger": "device_state_change"},
                "id": "trigger-node",
                "name": "Trigger IoT",
                "type": "n8n-nodes-base.webhook",
                "position": [240, 300],
            },
            {
                "parameters": {"condition": "anomaly_detected"},
                "id": "condition-node",
                "name": "Condición",
                "type": "n8n-nodes-base.if",
                "position": [440, 300],
            },
            {
                "parameters": {"action": "send_alert"},
                "id": "action-node",
                "name": "Alerta",
                "type": "n8n-nodes-base.gmail",
                "position": [640, 300],
            },
        ],
    )

    return {
        "devices": devices,
        "analysis": status_text,
        "dashboard": monitoring_dashboard["dashboard"]["url"],
        "workflow": alert_workflow,
    }


def main():
    """Ejecutar todos los workflows de ejemplo"""
    print("🎯 ATLAS WORKSPACE - Workflows de Automatización")
    print("Usando 5 herramientas gratuitas integradas")
    print("=" * 60)

    # Workflow 1: Trading
    trading_results = workflow_trading_automation()

    # Workflow 2: Inteligencia Clientes
    client_results = workflow_cliente_inteligencia()

    # Workflow 3: Monitoreo IoT
    iot_results = workflow_monitoring_iot()

    # Resumen final
    print("\n📊 RESUMEN DE WORKFLOWS")
    print("=" * 30)
    print(f"🤖 Análisis IA: {len(trading_results['analysis'])} caracteres")
    print(f"📈 Dashboards creados: 3")
    print(f"⚙️ Workflows n8n: 3")
    print(f"🏠 Dispositivos IoT: {len(iot_results['devices'].get('devices', []))}")
    print(f"🔗 Conexiones: Gmail configurado")

    # Estado general del sistema
    print(f"\n🎯 Estado General del Sistema:")
    status = tools.get_all_status()
    for key, value in status.items():
        if isinstance(value, dict):
            print(f"  {key}: {len(value)} elementos")
        else:
            print(f"  {key}: {value}")

    print(f"\n✅ Todos los workflows ejecutados exitosamente!")
    print(f"🌐 Acceso a herramientas:")
    print(f"  - n8n: http://localhost:5678")
    print(f"  - Appsmith: http://localhost")
    print(f"  - Home Assistant: http://localhost:8123")


if __name__ == "__main__":
    main()
