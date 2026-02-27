# 🚀 ATLAS NEXUS

**Professional Autonomous AI System**  
Multi-LLM Intelligence | Autonomous Planning | 50+ Professional Tools | Mobile Control

---

## 🎯 What is ATLAS NEXUS?

ATLAS NEXUS is a professional-grade autonomous AI system that combines multiple AI models (Ollama, DeepSeek, Claude, GPT) into a single, powerful platform. It can:

- 🧠 **Think Intelligently**: Automatically selects the best AI model for each task
- 🤖 **Act Autonomously**: Plans and executes multi-step goals without supervision
- 🛠️ **Use 50+ Tools**: Web scraping, file operations, data analysis, communications, and more
- 📱 **Control Remotely**: Full REST API + WebSocket for mobile/web control
- 🔄 **Self-Recover**: Automatically detects and fixes errors
- 💾 **Remember Everything**: Persistent memory and context management

---

## ⚡ Quick Start

### Prerequisites
- **Python 3.10+**
- **Ollama** (for local AI models)
- **Windows/Linux/Mac**

### Installation (Windows PowerShell)

```powershell
# 1. Clone or extract ATLAS NEXUS
cd C:\ATLAS_NEXUS

# 2. Run installation script
PowerShell -ExecutionPolicy Bypass -File install.ps1

# 3. Configure your API keys
notepad config\.env

# 4. Start ATLAS NEXUS
python nexus.py --mode api
```

### Quick Test

```bash
# Test the system
curl http://localhost:8000/status

# Ask Atlas to think
curl -X POST http://localhost:8000/think \
  -H "Content-Type: application/json" \
  -d '{"prompt": "Hello, are you operational?"}'

# Give Atlas a goal
curl -X POST http://localhost:8000/goal \
  -H "Content-Type: application/json" \
  -d '{"goal": "Search the web for latest AI news and summarize"}'
```

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   INTERFACES LAYER                       │
│  Dashboard | Mobile API | Telegram Bot | CLI             │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│              NEURAL ROUTER (Multi-LLM Brain)             │
│  Claude Sonnet 4 | DeepSeek | Ollama | GPT-4           │
│  Smart Selection | Fallback System | Response Cache      │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│           AUTONOMOUS ENGINE (Planning & Execution)       │
│  Multi-Step Planner | Async Executor | Auto-Recovery    │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│        TOOLS LAYER (50+ Professional Tools)              │
│  Web | Files | Database | API | Communication | System   │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│       MEMORY & PERSISTENCE (Vector DB | Graph DB)        │
└─────────────────────────────────────────────────────────┘
```

---

## 🔥 Key Features

### 1. **Hybrid Multi-LLM Intelligence**
- **Ollama + DeepSeek (Local)**: Fast, private, cost-free
- **Claude Sonnet 4**: Best for complex reasoning and planning
- **GPT-4**: Fallback for specific tasks
- **Smart Router**: Automatically picks the best model for each task

### 2. **True Autonomy**
```python
# Just give it a goal - it figures out the rest
await nexus.achieve_goal("Create a Python script to analyze this CSV file")

# Atlas will:
# 1. Plan the steps (read CSV, analyze, generate script)
# 2. Execute each step using appropriate tools
# 3. Handle errors and retry if needed
# 4. Return the completed result
```

### 3. **50+ Professional Tools**

**Web Tools:**
- Web search
- Web scraping
- Browser automation
- API integration

**File Tools:**
- Read/write files
- PDF processing
- Excel/CSV operations
- Image processing

**System Tools:**
- Command execution
- Process management
- System monitoring

**Communication:**
- Telegram bot
- Email sending
- SMS messaging

**Data Tools:**
- Data analysis
- Database operations
- Data visualization

### 4. **Full Remote Control**

**REST API:**
```bash
# All endpoints available at http://localhost:8000

GET  /status              # System status
POST /goal                # Create autonomous goal
GET  /goal/{id}           # Check goal progress
POST /think               # Direct AI query
GET  /tools               # List available tools
POST /tool/execute        # Execute specific tool
```

**WebSocket (Real-time):**
```javascript
const ws = new WebSocket('ws://localhost:8000/ws');
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Atlas update:', data);
};
```

### 5. **Auto-Recovery**
- Detects failures automatically
- Creates recovery plans
- Retries with alternative approaches
- Logs everything for debugging

---

## 📱 Mobile Control

ATLAS NEXUS is designed for remote control:

```python
# Your mobile app can:
1. Send goals: "POST /goal"
2. Monitor progress: WebSocket connection
3. Get notifications: Real-time updates
4. Execute tools: Direct tool calls
```

**Example Mobile Flow:**
1. User: "Create a report from this data"
2. App sends goal to ATLAS
3. ATLAS plans and executes
4. App receives real-time updates via WebSocket
5. App downloads completed report

---

## 🎨 Configuration

Edit `config/.env`:

```env
# Ollama (Local AI)
OLLAMA_HOST=http://localhost:11434

# OpenAI (Optional)
OPENAI_API_KEY=sk-...

# Anthropic Claude (Optional)
ANTHROPIC_API_KEY=sk-ant-...

# DeepSeek API (Optional)
DEEPSEEK_API_KEY=sk-...

# Telegram Bot (Optional)
TELEGRAM_BOT_TOKEN=...
TELEGRAM_OWNER_ID=...

# System
ATLAS_ROOT=C:/ATLAS_NEXUS
ATLAS_AUTONOMY=high
```

---

## 🚀 Usage Examples

### Example 1: Simple Query
```python
from nexus import AtlasNexus

nexus = AtlasNexus()
response = await nexus.think("What's the weather like?")
print(response)
```

### Example 2: Autonomous Goal
```python
plan = await nexus.achieve_goal(
    goal="Research the top 5 AI companies and create a comparison report",
    context={"format": "markdown"}
)

print(f"Plan ID: {plan.id}")
print(f"Status: {plan.status}")
print(f"Steps completed: {len([s for s in plan.steps if s.status == 'completed'])}")
```

### Example 3: Using Specific Tool
```python
result = await nexus.tools_registry.execute_tool(
    name="web_search",
    parameters={"query": "latest AI news", "max_results": 5},
    context={}
)
print(result)
```

### Example 4: API Call (Mobile App)
```javascript
// Send goal from mobile app
fetch('http://atlas-server:8000/goal', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({
    goal: 'Analyze my daily schedule and suggest optimizations',
    context: {schedule: dailySchedule}
  })
})
.then(r => r.json())
.then(data => {
  console.log('Plan ID:', data.plan_id);
  // Monitor via WebSocket
  monitorPlan(data.plan_id);
});
```

---

## 🛠️ Development

### Project Structure
```
atlas_nexus/
├── config/              # Configuration management
│   └── nexus_config.py
├── brain/               # AI Intelligence Layer
│   ├── neural_router.py
│   └── autonomous_engine.py
├── tools/               # Tools Registry
│   └── tools_registry.py
├── api/                 # REST API
│   └── rest_api.py
├── dashboard/           # Web Dashboard (React)
├── memory/              # Persistent Memory
├── logs/                # System Logs
├── snapshots/           # Backup Snapshots
└── nexus.py            # Main Entry Point
```

### Adding Custom Tools
```python
from tools.tools_registry import BaseTool, ToolMetadata, ToolCategory

class MyCustomTool(BaseTool):
    def __init__(self):
        super().__init__(ToolMetadata(
            name="my_tool",
            description="What my tool does",
            category=ToolCategory.CUSTOM,
            parameters={"param1": {"type": "string", "required": True}}
        ))
    
    async def execute(self, parameters, context):
        # Your tool logic here
        return {"result": "success"}

# Register it
nexus.tools_registry.register_tool(MyCustomTool())
```

---

## 📊 Monitoring

### Logs
```bash
# Real-time logs
tail -f logs/nexus.log

# View specific component
grep "brain.router" logs/nexus.log
```

### Metrics
Access metrics at `http://localhost:8000/status`

```json
{
  "status": "operational",
  "uptime": 3600,
  "active_plans": 2,
  "available_tools": 52,
  "active_connections": 3
}
```

---

## 🔒 Security

- API key authentication
- Encrypted sensitive data
- Audit logging
- IP whitelisting
- Tool approval system for risky operations

---

## 🐛 Troubleshooting

### Ollama Not Found
```bash
# Install Ollama
# Windows: https://ollama.ai/download
# Then pull models:
ollama pull deepseek-coder:6.7b
ollama pull deepseek-r1:latest
ollama pull llama3.2:latest
```

### Port Already in Use
```bash
# Change port in config/.env
ATLAS_API_PORT=8001
```

### Import Errors
```bash
# Reinstall dependencies
pip install -r requirements.txt --upgrade
```

---

## 📚 Documentation

- **API Docs**: `http://localhost:8000/docs`
- **ReDoc**: `http://localhost:8000/redoc`
- **WebSocket**: `ws://localhost:8000/ws`

---

## 🤝 Contributing

ATLAS NEXUS is designed to be extensible. You can:
- Add custom tools
- Create new AI model integrations
- Build custom dashboards
- Extend the API

---

## 📝 License

Professional License - For commercial use, please contact.

---

## 🙏 Credits

Built with:
- FastAPI
- Anthropic Claude
- OpenAI
- Ollama
- DeepSeek

---

## 📧 Support

For issues, questions, or feature requests, please create an issue or contact support.

---

**ATLAS NEXUS** - Where Intelligence Meets Automation 🚀
