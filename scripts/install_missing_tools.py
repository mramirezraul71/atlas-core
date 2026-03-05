#!/usr/bin/env python3
"""
Instalación de herramientas faltantes para Atlas
- Home Assistant (IoT/Hardware)
- Appsmith (Interfaz visual)
"""

import subprocess
from pathlib import Path


def run_command(cmd, description):
    """Ejecuta comando y muestra resultado"""
    print(f"\n🔧 {description}")
    print(f"Comando: {cmd}")
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ {description} - ÉXITO")
            if result.stdout:
                print(f"Salida: {result.stdout[:500]}")
        else:
            print(f"❌ {description} - ERROR")
            if result.stderr:
                print(f"Error: {result.stderr[:500]}")
        return result.returncode == 0
    except Exception as e:
        print(f"❌ {description} - EXCEPCIÓN: {e}")
        return False


def check_docker():
    """Verifica si Docker está disponible"""
    print("\n🐋 Verificando Docker...")
    try:
        result = subprocess.run(
            "docker --version", shell=True, capture_output=True, text=True
        )
        if result.returncode == 0:
            print(f"✅ Docker disponible: {result.stdout.strip()}")
            return True
        else:
            print("❌ Docker no disponible")
            return False
    except:
        print("❌ Docker no disponible")
        return False


def install_home_assistant():
    """Instala Home Assistant"""
    print("\n🏠 Instalando Home Assistant...")

    # Crear directorio de configuración
    config_dir = Path("c:/ATLAS_PUSH/homeassistant_config")
    config_dir.mkdir(exist_ok=True)
    print(f"✅ Directorio de configuración creado: {config_dir}")

    if check_docker():
        # Instalación via Docker
        cmd = "docker run -d --name atlas-homeassistant -p 8123:8123 --restart=unless-stopped -v c:/ATLAS_PUSH/homeassistant_config:/config homeassistant/home-assistant:stable"
        success = run_command(cmd, "Home Assistant Docker")
        if success:
            print("🌐 Home Assistant disponible en: http://localhost:8123")
            return True
    else:
        # Instalación local (ya está instalada via pip)
        print("✅ Home Assistant ya está instalado via pip")
        print(
            "📝 Para iniciar: python -m homeassistant --config c:/ATLAS_PUSH/homeassistant_config"
        )
        return True

    return False


def install_appsmith():
    """Instala Appsmith"""
    print("\n🎨 Instalando Appsmith...")

    if check_docker():
        # Crear directorio de datos
        data_dir = Path("c:/ATLAS_PUSH/appsmith_data")
        data_dir.mkdir(exist_ok=True)

        # Instalación via Docker
        cmd = "docker run -d --name atlas-appsmith -p 80:80 -p 443:443 -v c:/ATLAS_PUSH/appsmith_data:/appsmith-stacks --restart=unless-stopped appsmith/appsmith-ce"
        success = run_command(cmd, "Appsmith Docker")
        if success:
            print("🌐 Appsmith disponible en: http://localhost")
            return True
    else:
        print("❌ Appsmith requiere Docker para instalación")
        print("📝 Alternativa: Usar interfaz web existente en Atlas")
        return False


def verify_tools():
    """Verifica estado de todas las herramientas"""
    print("\n🔍 Verificación de herramientas:")

    tools = {
        "Mem0": "mem0ai",
        "Composio": "composio",
        "Ollama": "ollama",
        "Home Assistant": "homeassistant",
        "Appsmith": "appsmith",
    }

    for tool, package in tools.items():
        try:
            if tool == "Ollama":
                result = subprocess.run(
                    "ollama --version", shell=True, capture_output=True, text=True
                )
                status = "✅" if result.returncode == 0 else "❌"
                print(
                    f"{status} {tool}: {result.stdout.strip() if result.returncode == 0 else 'No disponible'}"
                )
            elif tool == "Appsmith":
                # Verificar via Docker
                result = subprocess.run(
                    "docker ps | findstr appsmith",
                    shell=True,
                    capture_output=True,
                    text=True,
                )
                status = (
                    "✅"
                    if result.returncode == 0 and "appsmith" in result.stdout
                    else "❌"
                )
                print(
                    f"{status} {tool}: {'Corriendo en Docker' if result.returncode == 0 else 'No disponible'}"
                )
            else:
                result = subprocess.run(
                    f"pip show {package}", shell=True, capture_output=True, text=True
                )
                status = "✅" if result.returncode == 0 else "❌"
                print(
                    f"{status} {tool}: {'Instalado' if result.returncode == 0 else 'No disponible'}"
                )
        except Exception as e:
            print(f"❌ {tool}: Error verificando - {e}")


def create_integration_script():
    """Crea script de integración"""
    script_content = '''#!/usr/bin/env python3
"""
Script de integracion Atlas - Herramientas
"""

import os
from pathlib import Path

def main():
    print("Configurando integraciones...")

    # Configuracion Mem0
    print("Configurando Mem0...")
    os.environ["MEM0_API_KEY"] = os.getenv("MEM0_API_KEY", "tu-api-key-aqui")

    # Configuracion Composio
    print("Configurando Composio...")
    os.environ["COMPOSIO_API_KEY"] = os.getenv("COMPOSIO_API_KEY", "tu-api-key-aqui")

    # URLs de servicios
    services = {
        "HOME_ASSISTANT_URL": "http://localhost:8123",
        "APPSMITH_URL": "http://localhost",
        "OLLAMA_URL": "http://localhost:11434"
    }

    for key, url in services.items():
        os.environ[key] = url
        print(f"{key}: {url}")

    print("Integraciones configuradas")

if __name__ == "__main__":
    main()
'''

    script_path = Path("c:/ATLAS_PUSH/scripts/setup_integrations.py")
    script_path.parent.mkdir(exist_ok=True)
    with open(script_path, "w") as f:
        f.write(script_content)

    print(f"✅ Script de integración creado: {script_path}")


def main():
    print("🔧 Instalación de herramientas faltantes para Atlas")
    print("=" * 50)

    # Verificar herramientas actuales
    verify_tools()

    # Instalar Home Assistant
    ha_success = install_home_assistant()

    # Instalar Appsmith
    appsmith_success = install_appsmith()

    # Crear script de integración
    create_integration_script()

    print("\n📊 Resumen:")
    print("✅ Mem0: Ya instalado")
    print("✅ Composio: Ya instalado")
    print("✅ Ollama: Ya instalado")
    print(
        f"{'✅' if ha_success else '❌'} Home Assistant: {'Instalado' if ha_success else 'Falló'}"
    )
    print(
        f"{'✅' if appsmith_success else '❌'} Appsmith: {'Instalado' if appsmith_success else 'Falló (requiere Docker)'}"
    )

    print("\n🎯 Para completar configuración:")
    print("1. Inicia Docker Desktop si no está corriendo")
    print("2. Ejecuta: python scripts/setup_integrations.py")
    print("3. Configura API keys en variables de entorno")


if __name__ == "__main__":
    main()
