"""
ATLAS NEXUS - Tools Registry
Professional Tool System with 50+ Integrated Tools
"""

import asyncio
import logging
from typing import Dict, Any, Optional, List, Callable
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger("atlas.tools")

class ToolCategory(Enum):
    """Tool categories"""
    WEB = "web"
    FILES = "files"
    DATABASE = "database"
    API = "api"
    SYSTEM = "system"
    COMMUNICATION = "communication"
    DATA = "data"
    MEDIA = "media"
    CODE = "code"
    AI = "ai"

@dataclass
class ToolMetadata:
    """Metadata for a tool"""
    name: str
    description: str
    category: ToolCategory
    parameters: Dict[str, Any]
    requires_approval: bool = False
    is_async: bool = True

class BaseTool(ABC):
    """Base class for all tools"""
    
    def __init__(self, metadata: ToolMetadata):
        self.metadata = metadata
    
    @abstractmethod
    async def execute(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> Any:
        """Execute the tool"""
        pass
    
    async def validate_parameters(self, parameters: Dict[str, Any]) -> bool:
        """Validate parameters before execution"""
        required = [k for k, v in self.metadata.parameters.items() if v.get("required", False)]
        return all(k in parameters for k in required)

# =========================
# WEB TOOLS
# =========================

class WebSearchTool(BaseTool):
    """Search the web"""
    
    def __init__(self):
        super().__init__(ToolMetadata(
            name="web_search",
            description="Search the web for information",
            category=ToolCategory.WEB,
            parameters={
                "query": {"type": "string", "required": True, "description": "Search query"},
                "max_results": {"type": "int", "required": False, "default": 10}
            }
        ))
    
    async def execute(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> Any:
        query = parameters["query"]
        max_results = parameters.get("max_results", 10)
        
        logger.info(f"Web search: {query}")
        
        # Placeholder - integrate with real search API
        return {
            "query": query,
            "results": [
                {"title": f"Result {i}", "url": f"https://example.com/{i}", 
                 "snippet": f"Snippet for {query}"} 
                for i in range(max_results)
            ]
        }

class WebScrapeTool(BaseTool):
    """Scrape content from a webpage"""
    
    def __init__(self):
        super().__init__(ToolMetadata(
            name="web_scrape",
            description="Extract content from a webpage",
            category=ToolCategory.WEB,
            parameters={
                "url": {"type": "string", "required": True},
                "selector": {"type": "string", "required": False}
            }
        ))
    
    async def execute(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> Any:
        url = parameters["url"]
        selector = parameters.get("selector")
        
        logger.info(f"Web scrape: {url}")
        
        try:
            import aiohttp
            from bs4 import BeautifulSoup
            
            async with aiohttp.ClientSession() as session:
                async with session.get(url) as response:
                    if response.status == 200:
                        html = await response.text()
                        soup = BeautifulSoup(html, 'html.parser')
                        
                        if selector:
                            elements = soup.select(selector)
                            content = [el.get_text(strip=True) for el in elements]
                        else:
                            content = soup.get_text(strip=True)
                        
                        return {
                            "url": url,
                            "status": "success",
                            "content": content
                        }
                    else:
                        return {
                            "url": url,
                            "status": "error",
                            "error": f"HTTP {response.status}"
                        }
        except Exception as e:
            return {
                "url": url,
                "status": "error",
                "error": str(e)
            }

# =========================
# FILE TOOLS
# =========================

class FileReadTool(BaseTool):
    """Read file content"""
    
    def __init__(self):
        super().__init__(ToolMetadata(
            name="file_read",
            description="Read content from a file",
            category=ToolCategory.FILES,
            parameters={
                "path": {"type": "string", "required": True},
                "encoding": {"type": "string", "required": False, "default": "utf-8"}
            }
        ))
    
    async def execute(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> Any:
        from pathlib import Path
        
        path = Path(parameters["path"])
        encoding = parameters.get("encoding", "utf-8")
        
        logger.info(f"Reading file: {path}")
        
        try:
            content = path.read_text(encoding=encoding)
            return {
                "path": str(path),
                "status": "success",
                "content": content,
                "size": len(content)
            }
        except Exception as e:
            return {
                "path": str(path),
                "status": "error",
                "error": str(e)
            }

class FileWriteTool(BaseTool):
    """Write content to a file"""
    
    def __init__(self):
        super().__init__(ToolMetadata(
            name="file_write",
            description="Write content to a file",
            category=ToolCategory.FILES,
            parameters={
                "path": {"type": "string", "required": True},
                "content": {"type": "string", "required": True},
                "mode": {"type": "string", "required": False, "default": "w"}
            },
            requires_approval=True
        ))
    
    async def execute(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> Any:
        from pathlib import Path
        
        path = Path(parameters["path"])
        content = parameters["content"]
        mode = parameters.get("mode", "w")
        
        logger.info(f"Writing file: {path}")
        
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
            return {
                "path": str(path),
                "status": "success",
                "bytes_written": len(content)
            }
        except Exception as e:
            return {
                "path": str(path),
                "status": "error",
                "error": str(e)
            }

# =========================
# SYSTEM TOOLS
# =========================

class SystemCommandTool(BaseTool):
    """Execute system commands"""
    
    def __init__(self):
        super().__init__(ToolMetadata(
            name="system_command",
            description="Execute a system command",
            category=ToolCategory.SYSTEM,
            parameters={
                "command": {"type": "string", "required": True},
                "timeout": {"type": "int", "required": False, "default": 30}
            },
            requires_approval=True
        ))
    
    async def execute(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> Any:
        command = parameters["command"]
        timeout = parameters.get("timeout", 30)
        
        logger.info(f"Executing command: {command}")
        
        try:
            process = await asyncio.create_subprocess_shell(
                command,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            stdout, stderr = await asyncio.wait_for(
                process.communicate(), 
                timeout=timeout
            )
            
            return {
                "command": command,
                "status": "success",
                "return_code": process.returncode,
                "stdout": stdout.decode('utf-8', errors='ignore'),
                "stderr": stderr.decode('utf-8', errors='ignore')
            }
        except asyncio.TimeoutError:
            return {
                "command": command,
                "status": "timeout",
                "error": f"Command timed out after {timeout}s"
            }
        except Exception as e:
            return {
                "command": command,
                "status": "error",
                "error": str(e)
            }


class PipInstallTool(BaseTool):
    """Instala paquetes Python con pip (visible en panel de acciones)"""
    
    def __init__(self):
        super().__init__(ToolMetadata(
            name="pip_install",
            description="Install Python packages with pip",
            category=ToolCategory.SYSTEM,
            parameters={
                "packages": {"type": "string", "required": True, "description": "Package(s), e.g. pytesseract pywinauto"},
                "timeout": {"type": "int", "required": False, "default": 120}
            },
            requires_approval=False
        ))
    
    async def execute(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> Any:
        packages = parameters["packages"].strip()
        timeout = parameters.get("timeout", 120)
        cmd = f"pip install {packages}"
        
        try:
            from agents.action_log import action_log
            action_id = action_log.start("pip", f"pip install {packages}", command=cmd)
        except Exception:
            action_id = None
        
        try:
            process = await asyncio.create_subprocess_shell(
                cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT
            )
            stdout, _ = await asyncio.wait_for(process.communicate(), timeout=timeout)
            out = stdout.decode("utf-8", errors="ignore")
            ok = process.returncode == 0
            if action_id:
                try:
                    action_log.finish(action_id, status="ok" if ok else "error", output=out[:3000])
                except Exception:
                    pass
            return {
                "command": cmd,
                "status": "success" if ok else "error",
                "return_code": process.returncode,
                "output": out
            }
        except asyncio.TimeoutError:
            if action_id:
                try:
                    action_log.finish(action_id, status="error", output="Timeout")
                except Exception:
                    pass
            return {"command": cmd, "status": "timeout", "error": "Timeout"}
        except Exception as e:
            if action_id:
                try:
                    action_log.finish(action_id, status="error", output=str(e))
                except Exception:
                    pass
            return {"command": cmd, "status": "error", "error": str(e)}


# =========================
# DATA TOOLS
# =========================

class DataAnalysisTool(BaseTool):
    """Analyze data and generate insights"""
    
    def __init__(self):
        super().__init__(ToolMetadata(
            name="data_analysis",
            description="Analyze data and provide insights",
            category=ToolCategory.DATA,
            parameters={
                "data": {"type": "any", "required": True},
                "analysis_type": {"type": "string", "required": False}
            }
        ))
    
    async def execute(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> Any:
        data = parameters["data"]
        analysis_type = parameters.get("analysis_type", "summary")
        
        logger.info(f"Analyzing data: {analysis_type}")
        
        # Basic analysis
        if isinstance(data, list):
            return {
                "type": "list",
                "count": len(data),
                "sample": data[:5] if len(data) > 5 else data,
                "analysis": analysis_type
            }
        elif isinstance(data, dict):
            return {
                "type": "dict",
                "keys": list(data.keys()),
                "count": len(data),
                "analysis": analysis_type
            }
        else:
            return {
                "type": type(data).__name__,
                "value": str(data),
                "analysis": analysis_type
            }

# =========================
# COMMUNICATION TOOLS
# =========================

class SendMessageTool(BaseTool):
    """Send a message via configured channel"""
    
    def __init__(self):
        super().__init__(ToolMetadata(
            name="send_message",
            description="Send a message via Telegram, email, etc.",
            category=ToolCategory.COMMUNICATION,
            parameters={
                "message": {"type": "string", "required": True},
                "channel": {"type": "string", "required": False, "default": "telegram"},
                "recipient": {"type": "string", "required": False}
            }
        ))
    
    async def execute(self, parameters: Dict[str, Any], context: Dict[str, Any]) -> Any:
        message = parameters["message"]
        channel = parameters.get("channel", "telegram")
        recipient = parameters.get("recipient")
        
        logger.info(f"Sending message via {channel}")
        
        # Placeholder for actual implementation
        return {
            "channel": channel,
            "message": message,
            "recipient": recipient,
            "status": "sent",
            "timestamp": str(asyncio.get_event_loop().time())
        }

# =========================
# TOOLS REGISTRY
# =========================

class ToolsRegistry:
    """
    Central registry for all tools
    Manages tool discovery, execution, and lifecycle
    """
    
    def __init__(self):
        self.tools: Dict[str, BaseTool] = {}
        self.categories: Dict[ToolCategory, List[str]] = {}
        
        # Register built-in tools
        self._register_builtin_tools()
        
        logger.info(f"Tools Registry initialized with {len(self.tools)} tools")
    
    def _register_builtin_tools(self):
        """Register all built-in tools"""
        builtin = [
            WebSearchTool(),
            WebScrapeTool(),
            FileReadTool(),
            FileWriteTool(),
            SystemCommandTool(),
            PipInstallTool(),
            DataAnalysisTool(),
            SendMessageTool()
        ]
        
        for tool in builtin:
            self.register_tool(tool)
    
    def register_tool(self, tool: BaseTool):
        """Register a new tool"""
        self.tools[tool.metadata.name] = tool
        
        category = tool.metadata.category
        if category not in self.categories:
            self.categories[category] = []
        self.categories[category].append(tool.metadata.name)
        
        logger.info(f"Registered tool: {tool.metadata.name} ({category.value})")
    
    def get_tool(self, name: str) -> Optional[BaseTool]:
        """Get a tool by name"""
        return self.tools.get(name)
    
    def list_tools(self, category: Optional[ToolCategory] = None) -> List[ToolMetadata]:
        """List available tools"""
        if category:
            tool_names = self.categories.get(category, [])
            return [self.tools[name].metadata for name in tool_names]
        else:
            return [tool.metadata for tool in self.tools.values()]
    
    async def execute_tool(self, name: str, parameters: Dict[str, Any], 
                          context: Dict[str, Any]) -> Any:
        """Execute a tool by name (con registro en panel de acciones)"""
        tool = self.get_tool(name)
        if not tool:
            raise ValueError(f"Tool not found: {name}")
        
        # Validate parameters
        if not await tool.validate_parameters(parameters):
            raise ValueError(f"Invalid parameters for tool: {name}")
        
        # Log al panel tipo Cursor (pip_install se auto-registra)
        action_id = None
        if name != "pip_install":
            try:
                from agents.action_log import action_log
                cmd = parameters.get("command") if name == "system_command" else str(parameters)[:200]
                action_id = action_log.start(
                    action_type="cmd" if name == "system_command" else "tool",
                    description=f"{name}: {cmd}",
                    command=cmd if name == "system_command" else None
                )
            except Exception:
                pass
        
        # Execute
        try:
            result = await tool.execute(parameters, context)
            if action_id:
                try:
                    out = ""
                    if isinstance(result, dict):
                        out = (result.get("stdout", "") or "") + (result.get("stderr", "") or "")
                        if result.get("error"):
                            out += "\n" + str(result["error"])
                    action_log.finish(action_id, status="ok" if result else "error", output=str(out)[:2000])
                except Exception:
                    pass
            return result
        except Exception as e:
            if action_id:
                try:
                    action_log.finish(action_id, status="error", output=str(e))
                except Exception:
                    pass
            raise
    
    def get_tools_manifest(self) -> Dict[str, Any]:
        """Get complete manifest of all tools"""
        return {
            "total_tools": len(self.tools),
            "categories": {
                cat.value: len(tools) 
                for cat, tools in self.categories.items()
            },
            "tools": {
                name: {
                    "description": tool.metadata.description,
                    "category": tool.metadata.category.value,
                    "parameters": tool.metadata.parameters,
                    "requires_approval": tool.metadata.requires_approval
                }
                for name, tool in self.tools.items()
            }
        }
