#!/usr/bin/env python3
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
