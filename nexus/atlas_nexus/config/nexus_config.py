"""
ATLAS NEXUS - Configuration Master
Professional AI System Configuration
"""

import os
from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass, field
from enum import Enum

class Environment(Enum):
    """Environment types"""
    DEVELOPMENT = "development"
    PRODUCTION = "production"
    TESTING = "testing"

class LLMProvider(Enum):
    """LLM Provider types"""
    OLLAMA = "ollama"
    DEEPSEEK = "deepseek"
    OPENAI = "openai"
    ANTHROPIC = "anthropic"
    GOOGLE = "google"

@dataclass
class PathConfig:
    """System paths configuration"""
    root: Path = Path(os.getenv("ATLAS_ROOT", "C:/ATLAS_NEXUS"))
    brain: Path = field(init=False)
    tools: Path = field(init=False)
    memory: Path = field(init=False)
    logs: Path = field(init=False)
    snapshots: Path = field(init=False)
    vault: Path = field(init=False)
    plugins: Path = field(init=False)
    
    def __post_init__(self):
        self.brain = self.root / "brain"
        self.tools = self.root / "tools"
        self.memory = self.root / "memory"
        self.logs = self.root / "logs"
        self.snapshots = self.root / "snapshots"
        self.vault = self.root / "ATLAS_VAULT"
        self.plugins = self.root / "plugins"
        
        # Create directories
        for path in [self.root, self.brain, self.tools, self.memory, 
                     self.logs, self.snapshots, self.vault, self.plugins]:
            path.mkdir(parents=True, exist_ok=True)

@dataclass
class OllamaConfig:
    """Ollama LLM Configuration"""
    enabled: bool = True
    host: str = "http://localhost:11434"
    models: Dict[str, str] = field(default_factory=lambda: {
        "coder": "deepseek-coder:6.7b",
        "chat": "deepseek-r1:latest",
        "fast": "llama3.2:latest",
        "embedding": "nomic-embed-text:latest",
        "reasoning": "deepseek-r1:14b"
    })
    timeout: int = 120
    temperature: float = 0.7
    max_tokens: int = 4096

@dataclass
class DeepSeekConfig:
    """DeepSeek Configuration"""
    enabled: bool = True
    api_key: Optional[str] = None
    models: Dict[str, str] = field(default_factory=lambda: {
        "coder": "deepseek-coder",
        "chat": "deepseek-chat",
        "reasoner": "deepseek-reasoner"
    })
    base_url: str = "https://api.deepseek.com"

@dataclass
class OpenAIConfig:
    """OpenAI Configuration"""
    enabled: bool = True
    api_key: Optional[str] = None
    models: Dict[str, str] = field(default_factory=lambda: {
        "gpt4": "gpt-4-turbo-preview",
        "gpt4mini": "gpt-4.1-mini",
        "embedding": "text-embedding-3-small"
    })

@dataclass
class AnthropicConfig:
    """Anthropic Claude Configuration"""
    enabled: bool = True
    api_key: Optional[str] = None
    models: Dict[str, str] = field(default_factory=lambda: {
        "sonnet": "claude-sonnet-4-20250514",
        "opus": "claude-opus-4-5-20251101",
        "haiku": "claude-haiku-4-5-20251001"
    })

@dataclass
class BrainConfig:
    """Brain/Router Configuration"""
    default_provider: LLMProvider = LLMProvider.OLLAMA
    fallback_chain: list = field(default_factory=lambda: [
        LLMProvider.OLLAMA,
        LLMProvider.OPENAI,
        LLMProvider.ANTHROPIC
    ])
    auto_select: bool = True
    cache_responses: bool = True
    cache_ttl: int = 3600
    max_retries: int = 3
    
@dataclass
class AutonomyConfig:
    """Autonomous behavior configuration"""
    level: str = "high"  # low, medium, high, full
    max_steps: int = 10
    require_approval: bool = False
    auto_recovery: bool = True
    self_improvement: bool = True
    learning_rate: float = 0.01

@dataclass
class ToolsConfig:
    """Tools system configuration"""
    enabled_categories: list = field(default_factory=lambda: [
        "web", "files", "database", "api", "system", "communication"
    ])
    web_driver: str = "playwright"  # selenium, playwright
    max_concurrent_tools: int = 5
    timeout: int = 300

@dataclass
class APIConfig:
    """API Server Configuration"""
    host: str = "0.0.0.0"
    port: int = 8000
    debug: bool = False
    cors_origins: list = field(default_factory=lambda: ["*"])
    websocket_enabled: bool = True
    api_key_required: bool = True
    rate_limit: int = 100  # requests per minute

@dataclass
class DashboardConfig:
    """Dashboard Configuration"""
    enabled: bool = True
    host: str = "localhost"
    port: int = 3000
    theme: str = "quantum_dark"  # quantum_dark, cyber_blue, matrix_green

@dataclass
class TelegramConfig:
    """Telegram Bot Configuration"""
    enabled: bool = True
    token: Optional[str] = None
    owner_id: Optional[str] = None
    webhook_url: Optional[str] = None

@dataclass
class SecurityConfig:
    """Security Configuration"""
    encryption_enabled: bool = True
    api_keys_encrypted: bool = True
    audit_log: bool = True
    allowed_ips: list = field(default_factory=lambda: ["127.0.0.1"])

class NexusConfig:
    """Master Configuration for ATLAS NEXUS"""
    
    def __init__(self, env: Environment = Environment.DEVELOPMENT):
        self.env = env
        self.paths = PathConfig()
        
        # LLM Configurations
        self.ollama = OllamaConfig()
        self.deepseek = DeepSeekConfig()
        self.openai = OpenAIConfig()
        self.anthropic = AnthropicConfig()
        
        # System Configurations
        self.brain = BrainConfig()
        self.autonomy = AutonomyConfig()
        self.tools = ToolsConfig()
        self.api = APIConfig()
        self.dashboard = DashboardConfig()
        self.telegram = TelegramConfig()
        self.security = SecurityConfig()
        
        # Load from environment
        self._load_from_env()
    
    def _load_from_env(self):
        """Load configuration from environment variables"""
        # OpenAI
        if api_key := os.getenv("OPENAI_API_KEY"):
            self.openai.api_key = api_key
        
        # Anthropic
        if api_key := os.getenv("ANTHROPIC_API_KEY"):
            self.anthropic.api_key = api_key
            
        # DeepSeek
        if api_key := os.getenv("DEEPSEEK_API_KEY"):
            self.deepseek.api_key = api_key
        
        # Telegram
        if token := os.getenv("TELEGRAM_BOT_TOKEN"):
            self.telegram.token = token
        if owner_id := os.getenv("TELEGRAM_OWNER_ID"):
            self.telegram.owner_id = owner_id
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary"""
        return {
            "environment": self.env.value,
            "paths": {
                "root": str(self.paths.root),
                "brain": str(self.paths.brain),
                "tools": str(self.paths.tools),
                "memory": str(self.paths.memory),
                "logs": str(self.paths.logs)
            },
            "llm_providers": {
                "ollama": self.ollama.enabled,
                "deepseek": self.deepseek.enabled,
                "openai": self.openai.enabled,
                "anthropic": self.anthropic.enabled
            },
            "autonomy_level": self.autonomy.level,
            "api_enabled": True,
            "dashboard_enabled": self.dashboard.enabled
        }

# Global configuration instance
config = NexusConfig()
