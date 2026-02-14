# 🛠️ ATLAS NEXUS - Development Guide

**Guide for extending and customizing ATLAS NEXUS**

---

## 📁 Project Structure

```
atlas_nexus/
├── config/
│   ├── nexus_config.py       # Configuration manager
│   └── __init__.py
├── brain/
│   ├── neural_router.py       # Multi-LLM orchestrator
│   ├── autonomous_engine.py   # Task planning & execution
│   └── __init__.py
├── tools/
│   ├── tools_manager.py       # Tools ecosystem
│   └── __init__.py
├── api/
│   ├── rest_api.py            # REST API server
│   └── __init__.py
├── dashboard/
│   └── index.html             # Web dashboard
├── modules/
│   └── (legacy modules)       # Migrated from ATLAS_PUSH
├── atlas_nexus.py             # Main entry point
├── requirements.txt           # Dependencies
├── .env.example               # Configuration template
├── install.ps1                # Windows installer
└── migrate.py                 # Migration tool
```

---

## 🔧 Adding a New Tool

### Step 1: Create Tool Class

Edit `tools/tools_manager.py`:

```python
class MyCustomTool(Tool):
    """Description of what this tool does"""
    def __init__(self):
        super().__init__(
            name="my_tool",
            description="What it does",
            category="category_name"  # web, files, code, data, etc.
        )
    
    async def execute(self, param1: str, param2: int = 10, **kwargs) -> Dict:
        """Execute tool logic"""
        try:
            # Your implementation here
            result = do_something(param1, param2)
            
            return {
                'result': result,
                'status': 'success'
            }
        except Exception as e:
            return {
                'error': str(e),
                'status': 'failed'
            }
```

### Step 2: Register Tool

In `ToolsManager._register_all_tools()`:

```python
def _register_all_tools(self):
    # ... existing tools ...
    
    # Add your tool
    self.register_tool(MyCustomTool())
```

### Step 3: Test Tool

```python
from tools.tools_manager import ToolsManager
from config.nexus_config import config
import asyncio

tools = ToolsManager(config)
result = asyncio.run(tools.execute('my_tool', {
    'param1': 'test',
    'param2': 20
}))
print(result)
```

---

## 🧠 Adding a New LLM Provider

### Step 1: Update Configuration

Edit `config/nexus_config.py`:

```python
def _init_llm_config(self) -> Dict[str, Any]:
    return {
        # ... existing providers ...
        
        'my_llm': {
            'enabled': bool(os.getenv('MY_LLM_API_KEY')),
            'api_key': os.getenv('MY_LLM_API_KEY', ''),
            'model': 'model-name',
            'base_url': 'https://api.example.com',
            'use_for': ['general', 'specific_task']
        }
    }
```

### Step 2: Implement Call Method

Edit `brain/neural_router.py`:

```python
async def _call_my_llm(self, prompt: str, system: Optional[str], 
                       temperature: float, max_tokens: int) -> Dict:
    """Call custom LLM provider"""
    # Your implementation
    response = requests.post(
        self.llm_config['my_llm']['base_url'],
        headers={'Authorization': f"Bearer {self.llm_config['my_llm']['api_key']}"},
        json={
            'prompt': prompt,
            'temperature': temperature,
            'max_tokens': max_tokens
        }
    )
    
    return {
        'content': response.json()['text'],
        'tokens': response.json()['usage']['total_tokens']
    }
```

### Step 3: Update Router Logic

In `_call_model()` method:

```python
async def _call_model(self, model: str, ...):
    # ... existing code ...
    
    elif model == 'my_llm':
        response = await self._call_my_llm(prompt, system, temperature, max_tokens)
```

---

## 🎨 Creating Custom Autonomous Workflows

### Example: Multi-Step Research Workflow

```python
from brain.autonomous_engine import AutonomousEngine, Task

async def research_workflow(engine: AutonomousEngine, topic: str):
    """Custom research workflow"""
    
    # Create main task
    task = Task(
        description=f"Research and analyze: {topic}",
        task_type='research'
    )
    
    # Custom execution plan
    plan = {
        'understanding': f'Research {topic}',
        'strategy': 'Multi-source analysis',
        'steps': [
            {
                'step_number': 1,
                'action': f'Search web for {topic}',
                'tool': 'web_search',
                'parameters': {'query': topic, 'max_results': 10}
            },
            {
                'step_number': 2,
                'action': 'Analyze search results',
                'tool': 'data_analyze',
                'parameters': {'data': '{step_1}'}
            },
            {
                'step_number': 3,
                'action': 'Create summary report',
                'tool': 'summarize',
                'parameters': {'text': '{step_2}'}
            }
        ]
    }
    
    # Execute
    result = await engine._execute_plan(task, plan)
    return result
```

---

## 📡 Adding Custom API Endpoints

### Step 1: Define Route

Edit `api/rest_api.py` in `_setup_routes()`:

```python
@self.app.post("/custom/endpoint")
async def custom_endpoint(
    data: Dict[str, Any],
    api_key: str = Depends(verify_api_key)
):
    """Custom endpoint description"""
    try:
        # Your logic here
        result = process_data(data)
        
        return {
            'success': True,
            'result': result
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

### Step 2: Test Endpoint

```bash
curl -X POST http://localhost:8000/custom/endpoint \
  -H "x-api-key: your-key" \
  -H "Content-Type: application/json" \
  -d '{"key": "value"}'
```

---

## 🔌 Integrating with External Services

### Example: Adding Slack Integration

```python
class SlackTool(Tool):
    """Send messages to Slack"""
    def __init__(self, config):
        super().__init__(
            name="slack_send",
            description="Send messages to Slack",
            category="communication"
        )
        self.webhook_url = os.getenv('SLACK_WEBHOOK_URL')
    
    async def execute(self, message: str, channel: str = None, **kwargs) -> Dict:
        """Send Slack message"""
        try:
            payload = {
                'text': message,
                'channel': channel
            }
            
            response = requests.post(
                self.webhook_url,
                json=payload
            )
            
            return {
                'success': response.ok,
                'status': 'success' if response.ok else 'failed'
            }
        except Exception as e:
            return {'error': str(e), 'status': 'failed'}
```

---

## 🧪 Testing

### Unit Tests

Create `tests/test_tools.py`:

```python
import pytest
import asyncio
from tools.tools_manager import ToolsManager
from config.nexus_config import config

@pytest.mark.asyncio
async def test_web_search():
    tools = ToolsManager(config)
    result = await tools.execute('web_search', {'query': 'test', 'max_results': 3})
    assert result['status'] == 'success'
    assert len(result) > 0

@pytest.mark.asyncio
async def test_file_operations():
    tools = ToolsManager(config)
    
    # Write file
    write_result = await tools.execute('file_write', {
        'path': '/tmp/test.txt',
        'content': 'test content'
    })
    assert write_result['status'] == 'success'
    
    # Read file
    read_result = await tools.execute('file_read', {
        'path': '/tmp/test.txt'
    })
    assert read_result['content'] == 'test content'
```

Run tests:
```bash
pytest tests/
```

---

## 📊 Performance Optimization

### Caching Results

```python
from functools import lru_cache
import hashlib

class CachedTool(Tool):
    def __init__(self):
        super().__init__(name="cached_tool", ...)
        self.cache = {}
    
    async def execute(self, query: str, **kwargs):
        # Create cache key
        key = hashlib.md5(query.encode()).hexdigest()
        
        # Check cache
        if key in self.cache:
            return self.cache[key]
        
        # Execute
        result = await self._expensive_operation(query)
        
        # Cache result
        self.cache[key] = result
        return result
```

### Parallel Execution

```python
async def parallel_tasks():
    tasks = [
        engine.execute("task 1"),
        engine.execute("task 2"),
        engine.execute("task 3")
    ]
    
    results = await asyncio.gather(*tasks)
    return results
```

---

## 🔒 Security Best Practices

1. **Never commit `.env` files**
   ```bash
   # Add to .gitignore
   .env
   *.env
   config/.env
   ```

2. **Use environment variables**
   ```python
   api_key = os.getenv('API_KEY')  # Good
   api_key = "hardcoded-key"       # Bad!
   ```

3. **Validate inputs**
   ```python
   def validate_path(path: str):
       # Prevent directory traversal
       safe_path = Path(path).resolve()
       if not str(safe_path).startswith('/allowed/directory'):
           raise ValueError("Invalid path")
   ```

4. **Rate limiting**
   ```python
   from fastapi_limiter import FastAPILimiter
   from fastapi_limiter.depends import RateLimiter
   
   @app.post("/task/execute")
   @limiter.limit("10/minute")
   async def execute_task(...):
       ...
   ```

---

## 📚 Resources

- FastAPI docs: https://fastapi.tiangolo.com
- Anthropic API: https://docs.anthropic.com
- Ollama: https://github.com/jmorganca/ollama
- Python Telegram Bot: https://python-telegram-bot.org

---

## 💡 Tips & Tricks

### 1. Debug Mode
```python
# Enable detailed logging
import logging
logging.basicConfig(level=logging.DEBUG)
```

### 2. Hot Reload
```bash
# API server with auto-reload
uvicorn api.rest_api:app --reload
```

### 3. Interactive Development
```python
# IPython for testing
ipython
>>> from brain.neural_router import NeuralRouter
>>> from config.nexus_config import config
>>> router = NeuralRouter(config)
>>> await router.think("test")
```

---

## 🤝 Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature-name`
3. Make changes
4. Test thoroughly
5. Submit pull request

---

<p align="center">
  <strong>Happy Building! 🚀</strong>
</p>
