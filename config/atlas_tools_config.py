"""
Configuracion centralizada de herramientas para Atlas
"""

import os
from pathlib import Path

class AtlasToolsConfig:
    """Configuracion de todas las herramientas integradas en Atlas"""
    
    def __init__(self):
        self.base_dir = Path("c:/ATLAS_PUSH")
        
        # URLs de servicios
        self.HOME_ASSISTANT_URL = "http://localhost:8123"
        self.APPSMITH_URL = "http://localhost"
        self.OLLAMA_URL = "http://localhost:11434"
        
        # Configuraciones de API
        self.MEM0_API_KEY = os.getenv("MEM0_API_KEY", "")
        self.COMPOSIO_API_KEY = os.getenv("COMPOSIO_API_KEY", "")
        
        # Directorios de configuracion
        self.HOME_ASSISTANT_CONFIG = self.base_dir / "homeassistant_config"
        self.APPSMITH_DATA = self.base_dir / "appsmith_data"
        
        # Modelos Ollama disponibles
        self.OLLAMA_MODELS = [
            "llama2",
            "mistral", 
            "codellama",
            "phi3"
        ]
        
    def setup_environment(self):
        """Configura variables de entorno"""
        os.environ["HOME_ASSISTANT_URL"] = self.HOME_ASSISTANT_URL
        os.environ["APPSMITH_URL"] = self.APPSMITH_URL
        os.environ["OLLAMA_URL"] = self.OLLAMA_URL
        
        if self.MEM0_API_KEY:
            os.environ["MEM0_API_KEY"] = self.MEM0_API_KEY
            
        if self.COMPOSIO_API_KEY:
            os.environ["COMPOSIO_API_KEY"] = self.COMPOSIO_API_KEY
            
    def verify_services(self):
        """Verifica estado de los servicios"""
        import requests
        import subprocess
        
        status = {}
        
        # Verificar Home Assistant
        try:
            response = requests.get(f"{self.HOME_ASSISTANT_URL}", timeout=5)
            status["home_assistant"] = response.status_code == 200
        except:
            status["home_assistant"] = False
            
        # Verificar Appsmith
        try:
            response = requests.get(f"{self.APPSMITH_URL}", timeout=5)
            status["appsmith"] = response.status_code == 200
        except:
            status["appsmith"] = False
            
        # Verificar Ollama
        try:
            result = subprocess.run(["ollama", "--version"], capture_output=True, text=True)
            status["ollama"] = result.returncode == 0
        except:
            status["ollama"] = False
            
        # Verificar Mem0
        try:
            import mem0
            status["mem0"] = True
        except ImportError:
            status["mem0"] = False
            
        # Verificar Composio
        try:
            import composio
            status["composio"] = True
        except ImportError:
            status["composio"] = False
            
        return status
        
    def print_status(self):
        """Muestra estado actual de todas las herramientas"""
        status = self.verify_services()
        
        print("=== Estado de Herramientas Atlas ===")
        print(f"Mem0 (Memoria): {'✅ Activo' if status['mem0'] else '❌ Inactivo'}")
        print(f"Composio (Conectividad): {'✅ Activo' if status['composio'] else '❌ Inactivo'}")
        print(f"Ollama (Motor IA): {'✅ Activo' if status['ollama'] else '❌ Inactivo'}")
        print(f"Home Assistant (IoT): {'✅ Activo' if status['home_assistant'] else '❌ Inactivo'}")
        print(f"Appsmith (Interfaz): {'✅ Activo' if status['appsmith'] else '❌ Inactivo'}")
        
        print("\n=== URLs de Acceso ===")
        print(f"Home Assistant: {self.HOME_ASSISTANT_URL}")
        print(f"Appsmith: {self.APPSMITH_URL}")
        print(f"Ollama API: {self.OLLAMA_URL}")

# Instancia global
config = AtlasToolsConfig()

if __name__ == "__main__":
    config.setup_environment()
    config.print_status()
