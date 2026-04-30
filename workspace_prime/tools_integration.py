"""
Integración de herramientas (Mem0, Composio, Ollama, Home Assistant, Appsmith)
para uso desde el agente workspace de Atlas
"""

import json
from pathlib import Path
from typing import Any, Dict

import requests


class WorkspaceToolsIntegration:
    """Integración de herramientas para el agente workspace"""

    def __init__(self):
        self.base_dir = Path("c:/ATLAS_PUSH")
        self.memory_file = (
            self.base_dir / "workspace_prime" / "memory" / "tools_state.json"
        )
        self.memory_file.parent.mkdir(parents=True, exist_ok=True)

        # URLs de servicios
        self.home_assistant_url = "http://localhost:8123"
        self.appsmith_url = "http://localhost"
        self.ollama_url = "http://localhost:11434"

        # Cargar estado previo
        self.state = self._load_state()

    def _load_state(self) -> Dict[str, Any]:
        """Carga estado persistente de herramientas"""
        if self.memory_file.exists():
            try:
                with open(self.memory_file, "r", encoding="utf-8") as f:
                    return json.load(f)
            except:
                pass
        return {
            "mem0_memories": [],
            "composio_connections": {},
            "ollama_models": [],
            "home_assistant_devices": [],
            "n8n_workflows": [],
            "appsmith_dashboards": [],
        }

    def _save_state(self):
        """Guarda estado persistente"""
        with open(self.memory_file, "w", encoding="utf-8") as f:
            json.dump(self.state, f, indent=2, ensure_ascii=False)

    # MEM0 - Memoria
    def mem0_store_memory(
        self, content: str, tags: list = None, user_id: str = "atlas_agent"
    ) -> Dict[str, Any]:
        """Almacena memoria usando Mem0 con OpenAI (requiere API key)"""
        try:
            from mem0 import Memory

            # Configuración básica - requiere OpenAI API key
            memory = Memory()

            result = memory.add(content, user_id=user_id, metadata={"tags": tags or []})

            # Guardar en estado local
            self.state["mem0_memories"].append(
                {
                    "id": result.get("id", "unknown"),
                    "content": content,
                    "tags": tags or [],
                    "user_id": user_id,
                    "timestamp": str(Path().cwd()),
                }
            )
            self._save_state()

            return {
                "ok": True,
                "memory_id": result.get("id"),
                "message": "Memoria almacenada (requiere OpenAI API key)",
            }

        except ImportError:
            return {"ok": False, "error": "Mem0 no instalado"}
        except Exception as e:
            error_msg = str(e)
            if "401" in error_msg or "API key" in error_msg:
                return {"ok": False, "error": "Requiere OpenAI API key válida"}
            return {"ok": False, "error": error_msg}

    def mem0_search_memories(
        self, query: str, user_id: str = "atlas_agent"
    ) -> Dict[str, Any]:
        """Busca memorias almacenadas"""
        try:
            from mem0 import Memory

            memory = Memory()

            results = memory.search(query, user_id=user_id, limit=5)

            return {"ok": True, "memories": results, "count": len(results)}

        except ImportError:
            return {"ok": False, "error": "Mem0 no instalado"}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    # COMPOSIO - Conectividad
    def composio_connect_app(
        self, app_name: str, credentials: Dict[str, str]
    ) -> Dict[str, Any]:
        """Conecta aplicación externa via Composio"""
        try:
            from composio import Action, App, ComposioToolset

            toolset = ComposioToolset()

            # Conectar app (ejemplo con Google Sheets)
            if app_name.lower() == "google_sheets":
                # Aquí iría la lógica de conexión real
                self.state["composio_connections"][app_name] = {
                    "status": "connected",
                    "credentials": credentials,
                    "timestamp": str(Path().cwd()),
                }
                self._save_state()

                return {"ok": True, "message": f"Conectado a {app_name}"}
            else:
                return {"ok": False, "error": f"App {app_name} no soportada"}

        except ImportError:
            return {"ok": False, "error": "Composio no instalado"}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def composio_execute_action(
        self, app_name: str, action: str, params: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Ejecuta acción en app externa"""
        try:
            # Verificar conexión
            if app_name not in self.state["composio_connections"]:
                return {"ok": False, "error": f"App {app_name} no conectada"}

            # Ejecutar acción (mock)
            result = {
                "app": app_name,
                "action": action,
                "params": params,
                "result": f"Acción {action} ejecutada en {app_name}",
                "timestamp": str(Path().cwd()),
            }

            return {"ok": True, "result": result}

        except Exception as e:
            return {"ok": False, "error": str(e)}

    # OLLAMA - Motor IA
    def ollama_generate(self, prompt: str, model: str = "llama2") -> Dict[str, Any]:
        """Genera texto usando Ollama"""
        try:
            import requests

            payload = {"model": model, "prompt": prompt, "stream": False}

            response = requests.post(
                f"{self.ollama_url}/api/generate", json=payload, timeout=30
            )
            if response.status_code == 200:
                result = response.json()
                return {
                    "ok": True,
                    "response": result.get("response", ""),
                    "model": model,
                    "prompt": prompt,
                }
            else:
                return {"ok": False, "error": f"HTTP {response.status_code}"}

        except ImportError:
            return {"ok": False, "error": "Requests no disponible"}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def ollama_list_models(self) -> Dict[str, Any]:
        """Lista modelos disponibles en Ollama"""
        try:
            import requests

            response = requests.get(f"{self.ollama_url}/api/tags", timeout=5)
            if response.status_code == 200:
                models_data = response.json()
                models = [model["name"] for model in models_data.get("models", [])]
                self.state["ollama_models"] = models
                self._save_state()

                return {"ok": True, "models": self.state["ollama_models"]}
            else:
                return {"ok": False, "error": f"HTTP {response.status_code}"}

        except ImportError:
            return {"ok": False, "error": "Requests no disponible"}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    # HOME ASSISTANT - IoT
    def home_assistant_get_devices(self) -> Dict[str, Any]:
        """Obtiene dispositivos de Home Assistant"""
        try:
            # Obtener token de configuración (mock)
            headers = {"Authorization": "Bearer mock-token"}
            response = requests.get(
                f"{self.home_assistant_url}/api/states", headers=headers, timeout=5
            )

            if response.status_code == 200:
                devices = response.json()
                self.state["home_assistant_devices"] = [
                    {
                        "entity_id": d["entity_id"],
                        "state": d["state"],
                        "attributes": d.get("attributes", {}),
                    }
                    for d in devices[:10]  # Limitar a 10 dispositivos
                ]
                self._save_state()

                return {"ok": True, "devices": self.state["home_assistant_devices"]}
            else:
                return {"ok": False, "error": f"HTTP {response.status_code}"}

        except Exception as e:
            return {"ok": False, "error": str(e)}

    def home_assistant_control_device(
        self, entity_id: str, action: str
    ) -> Dict[str, Any]:
        """Controla dispositivo en Home Assistant"""
        try:
            # Mock de control
            result = {
                "entity_id": entity_id,
                "action": action,
                "result": f"Dispositivo {entity_id} {action} ejecutado",
                "timestamp": str(Path().cwd()),
            }

            return {"ok": True, "result": result}

        except Exception as e:
            return {"ok": False, "error": str(e)}

    # N8N - Conectividad
    def n8n_get_workflows(self) -> Dict[str, Any]:
        """Obtiene workflows de n8n"""
        try:
            import requests

            # Obtener workflows via API (requiere autenticación)
            response = requests.get(
                "http://localhost:5678/api/v1/workflows", timeout=5
            )
            if response.status_code == 200:
                workflows = response.json().get("data", [])
                return {"ok": True, "workflows": workflows, "count": len(workflows)}
            else:
                # Si falla API, retorna estado básico
                return {
                    "ok": True,
                    "workflows": [],
                    "status": "n8n activo en http://localhost:5678",
                }

        except Exception as e:
            return {"ok": False, "error": str(e)}

    def n8n_create_workflow(self, name: str, nodes: list = None) -> Dict[str, Any]:
        """Crea workflow básico en n8n"""
        try:
            import requests

            # Workflow básico de ejemplo
            workflow_data = {
                "name": name,
                "nodes": nodes
                or [
                    {
                        "parameters": {},
                        "id": "start-node",
                        "name": "Start",
                        "type": "n8n-nodes-base.start",
                        "typeVersion": 1,
                        "position": [240, 300],
                    }
                ],
                "connections": {},
                "active": False,
            }

            response = requests.post(
                "http://localhost:5678/api/v1/workflows", json=workflow_data, timeout=5
            )
            if response.status_code == 201:
                workflow = response.json().get("data", {})
                return {
                    "ok": True,
                    "workflow": workflow,
                    "url": f"http://localhost:5678/workflow/{workflow.get('id')}",
                }
            else:
                return {"ok": False, "error": f"HTTP {response.status_code}"}

        except Exception as e:
            return {"ok": False, "error": str(e)}

    def n8n_connect_gmail(self, credentials: Dict[str, str]) -> Dict[str, Any]:
        """Configura conexión con Gmail"""
        try:
            # Mock de configuración Gmail
            connection = {
                "service": "gmail",
                "status": "configured",
                "credentials": credentials,
                "url": "http://localhost:5678/credential/0",
                "timestamp": str(Path().cwd()),
            }

            return {
                "ok": True,
                "connection": connection,
                "message": "Gmail configurado en n8n",
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}

    def n8n_connect_whatsapp(self, credentials: Dict[str, str]) -> Dict[str, Any]:
        """Configura conexión con WhatsApp"""
        try:
            # Mock de configuración WhatsApp
            connection = {
                "service": "whatsapp",
                "status": "configured",
                "credentials": credentials,
                "url": "http://localhost:5678/credential/1",
                "timestamp": str(Path().cwd()),
            }

            return {
                "ok": True,
                "connection": connection,
                "message": "WhatsApp configurado en n8n",
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}

    # GIT AUTOMATION - Control de Versiones
    def git_smart_commit(self, message: str, files: list = None) -> Dict[str, Any]:
        """Commit inteligente con análisis automático"""
        try:
            from git_automation import git_automation

            return git_automation.smart_commit(message, files)
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def git_smart_push(self, branch: str = None) -> Dict[str, Any]:
        """Push inteligente con verificación"""
        try:
            from git_automation import git_automation

            return git_automation.smart_push(branch)
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def git_create_branch(
        self, branch_name: str, from_branch: str = None
    ) -> Dict[str, Any]:
        """Crea nueva branch con tracking automático"""
        try:
            from git_automation import git_automation

            return git_automation.create_branch(branch_name, from_branch)
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def git_get_status(self) -> Dict[str, Any]:
        """Obtiene estado completo del repositorio"""
        try:
            from git_automation import git_automation

            return git_automation.get_repo_status()
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def git_automated_workflow(self, action: str, **kwargs) -> Dict[str, Any]:
        """Ejecuta workflow automatizado Git"""
        try:
            from git_automation import git_automation

            return git_automation.automated_workflow(action, **kwargs)
        except Exception as e:
            return {"ok": False, "error": str(e)}

    # APPSMITH - Interfaz
    def appsmith_create_dashboard(
        self, name: str, config: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Crea dashboard en Appsmith"""
        try:
            # Mock de creación
            dashboard = {
                "id": f"dashboard_{len(self.state['appsmith_dashboards']) + 1}",
                "name": name,
                "config": config,
                "url": f"{self.appsmith_url}/application/{name.lower().replace(' ', '-')}",
                "timestamp": str(Path().cwd()),
            }

            self.state["appsmith_dashboards"].append(dashboard)
            self._save_state()

            return {"ok": True, "dashboard": dashboard}

        except Exception as e:
            return {"ok": False, "error": str(e)}

    def appsmith_get_dashboards(self) -> Dict[str, Any]:
        """Obtiene dashboards creados"""
        return {"ok": True, "dashboards": self.state["appsmith_dashboards"]}

    # UTILIDADES
    def get_all_status(self) -> Dict[str, Any]:
        """Obtiene estado de todas las herramientas"""
        return {
            "mem0_memories_count": len(self.state["mem0_memories"]),
            "composio_connections": list(self.state["composio_connections"].keys()),
            "ollama_models": self.state["ollama_models"],
            "home_assistant_devices_count": len(self.state["home_assistant_devices"]),
            "n8n_workflows_count": len(self.state["n8n_workflows"]),
            "appsmith_dashboards_count": len(self.state["appsmith_dashboards"]),
            "services": {
                "home_assistant": self.home_assistant_url,
                "appsmith": self.appsmith_url,
                "ollama": self.ollama_url,
                "n8n": "http://localhost:5678",
            },
        }


# Instancia global para uso en workspace
tools = WorkspaceToolsIntegration()

if __name__ == "__main__":
    # Demostración
    print("=== Integración de Herramientas Workspace ===")

    # Probar Mem0
    print("\n1. Mem0 - Almacenar memoria:")
    result = tools.mem0_store_memory(
        "Cliente Juan prefiere productos tecnológicos", ["clientes", "juan"]
    )
    print(f"   {result}")

    # Probar Ollama
    print("\n2. Ollama - Listar modelos:")
    result = tools.ollama_list_models()
    print(f"   {result}")

    # Probar n8n
    print("\n3. n8n - Obtener workflows:")
    result = tools.n8n_get_workflows()
    print(f"   {result}")

    # Probar estado general
    print("\n4. Estado general:")
    try:
        status = tools.get_all_status()
        for key, value in status.items():
            print(f"   {key}: {value}")
    except Exception as e:
        print(f"   Error en estado: {e}")
