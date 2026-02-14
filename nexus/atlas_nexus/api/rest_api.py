"""
ATLAS NEXUS - REST API
Professional FastAPI server for remote control and mobile integration
"""

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, Depends, Header
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
import asyncio
import logging
from datetime import datetime

logger = logging.getLogger("atlas.api")

# =========================
# MODELS
# =========================

class GoalRequest(BaseModel):
    """Request to achieve a goal"""
    goal: str = Field(..., description="Goal to achieve")
    context: Optional[Dict[str, Any]] = Field(default=None, description="Additional context")

class ThinkRequest(BaseModel):
    """Request for AI thinking"""
    prompt: str = Field(..., description="Prompt for the AI")
    task_type: str = Field(default="conversation", description="Type of task")
    system_prompt: Optional[str] = Field(default=None, description="System prompt")
    temperature: float = Field(default=0.7, ge=0.0, le=2.0)

class ToolExecutionRequest(BaseModel):
    """Request to execute a tool"""
    tool_name: str = Field(..., description="Name of the tool to execute")
    parameters: Dict[str, Any] = Field(..., description="Tool parameters")

# =========================
# WEBSOCKET MANAGER
# =========================

class ConnectionManager:
    """Manage WebSocket connections"""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
    
    async def broadcast(self, message: Dict[str, Any]):
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except:
                pass

# =========================
# API CREATOR
# =========================

def create_api(nexus_engine) -> FastAPI:
    """Create FastAPI application"""
    app = FastAPI(title="ATLAS NEXUS API", version="1.0.0")
    
    # Importar y registrar router de directivas
    try:
        from directives.directives_api import register_directives_api
        register_directives_api(app)
        logger.info("Directives API registered successfully")
    except Exception as e:
        logger.warning(f"Could not register Directives API: {e}")
    
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    
    ws_manager = ConnectionManager()
    start_time = datetime.now()

    # Panel de acciones tipo Cursor: broadcast al WebSocket
    try:
        from agents.action_log import action_log
        def broadcast_action(payload):
            try:
                loop = asyncio.get_running_loop()
                loop.create_task(ws_manager.broadcast(payload))
            except RuntimeError:
                pass
        action_log.set_broadcast(broadcast_action)
    except Exception as e:
        logger.warning(f"Action log broadcast not available: {e}")
    
    @app.get("/")
    async def root():
        return {"service": "ATLAS NEXUS", "status": "operational", "dashboard": "/dashboard"}

    @app.get("/health")
    async def health():
        """Health para CEREBRO (ATLAS_PUSH) heartbeat en puerto 8000."""
        return {"status": "healthy", "service": "ATLAS NEXUS"}

    @app.get("/dashboard")
    async def serve_dashboard():
        """Sirve el panel tipo Cursor con operaciones en vivo"""
        from pathlib import Path
        p = Path(__file__).parent.parent / "dashboard" / "index.html"
        if p.exists():
            return FileResponse(p)
        raise HTTPException(status_code=404, detail="Dashboard not found")
    
    @app.get("/status")
    async def get_status():
        uptime = (datetime.now() - start_time).total_seconds()
        active = len(nexus_engine.autonomous_engine.active_plans)
        return {
            "status": "operational",
            "version": "2.0.0",
            "uptime": uptime,
            "active_plans": active,
            "active_tasks": active,
            "completed_tasks": 0,
            "available_tools": len(nexus_engine.tools_registry.tools)
        }
    
    @app.post("/goal")
    async def create_goal(request: GoalRequest):
        try:
            plan = await nexus_engine.autonomous_engine.achieve_goal(
                goal=request.goal,
                context=request.context or {}
            )
            
            await ws_manager.broadcast({
                "type": "goal_created",
                "plan_id": plan.id,
                "goal": plan.goal
            })
            
            return {
                "plan_id": plan.id,
                "goal": plan.goal,
                "status": plan.status.value,
                "steps_count": len(plan.steps)
            }
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))
    
    @app.get("/goal/{plan_id}")
    async def get_goal_status(plan_id: str):
        plan = nexus_engine.autonomous_engine.get_plan_status(plan_id)
        if not plan:
            raise HTTPException(status_code=404, detail="Plan not found")
        
        return {
            "plan_id": plan.id,
            "goal": plan.goal,
            "status": plan.status.value,
            "steps": [{
                "id": s.id,
                "description": s.description,
                "status": s.status.value
            } for s in plan.steps]
        }
    
    @app.post("/think")
    async def think(request: ThinkRequest):
        from brain.neural_router import TaskContext, TaskType
        
        try:
            task_type = TaskType[request.task_type.upper()]
        except:
            task_type = TaskType.CONVERSATION
        
        task_ctx = TaskContext(
            prompt=request.prompt,
            task_type=task_type,
            system_prompt=request.system_prompt,
            temperature=request.temperature
        )
        
        response = await nexus_engine.router.think(task_ctx)
        
        return {
            "content": response.content,
            "model": response.model,
            "provider": response.provider
        }
    
    @app.get("/tools")
    async def list_tools():
        return nexus_engine.tools_registry.get_tools_manifest()

    @app.get("/actions/log")
    async def get_actions_log(limit: int = 50):
        """Log de acciones (pip, comandos, etc.) para panel tipo Cursor"""
        try:
            from agents.action_log import action_log
            return {"ok": True, "entries": action_log.get_entries(limit=limit)}
        except Exception as e:
            return {"ok": False, "entries": [], "error": str(e)}
    
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await ws_manager.connect(websocket)
        try:
            await websocket.send_json({"type": "connected", "message": "ATLAS NEXUS"})
            while True:
                data = await websocket.receive_json()
                await ws_manager.broadcast({"type": "message", "data": data})
        except WebSocketDisconnect:
            ws_manager.disconnect(websocket)
    
    return app
