# 🚀 ATLAS NEXUS - Quick Start Guide

**Get up and running in 5 minutes!**

---

## ⚡ Super Quick Start (Windows)

```powershell
# 1. Open PowerShell as Administrator and run:
powershell -ExecutionPolicy Bypass -File install.ps1

# 2. Edit your API keys:
notepad C:\ATLAS\config\.env

# 3. Activate and test:
.\venv\Scripts\Activate.ps1
python atlas_nexus.py
```

That's it! You're ready to use ATLAS NEXUS.

---

## 📋 What You Need

### Required
- ✅ Python 3.10 or higher
- ✅ At least one API key:
  - Anthropic (Claude): https://console.anthropic.com
  - OpenAI (GPT): https://platform.openai.com

### Optional (but recommended)
- 🔧 Ollama: https://ollama.ai (for free local models)
- 💬 Telegram Bot: https://t.me/BotFather (for mobile control)

---

## 🎯 Step-by-Step Setup

### Step 1: Install Ollama (Optional but Free!)

Download and install Ollama from https://ollama.ai

Then pull the models:
```bash
ollama pull deepseek-coder:latest
ollama pull deepseek-chat:latest
ollama pull llama3.2:latest
```

This gives you **free, unlimited, local AI models**!

### Step 2: Get API Keys

1. **Claude (Recommended)**:
   - Go to https://console.anthropic.com
   - Create account → Get API key
   - Copy the key starting with `sk-ant-`

2. **OpenAI (Optional)**:
   - Go to https://platform.openai.com
   - Create account → Get API key
   - Copy the key starting with `sk-`

### Step 3: Configure ATLAS NEXUS

Edit `C:\ATLAS\config\.env`:

```bash
# Add your API keys
ANTHROPIC_API_KEY=sk-ant-your-key-here
OPENAI_API_KEY=sk-your-key-here

# For Telegram (optional)
TELEGRAM_BOT_TOKEN=your-bot-token
TELEGRAM_OWNER_ID=your-user-id
```

### Step 4: Test Your Installation

```bash
# Activate virtual environment
.\venv\Scripts\Activate.ps1  # Windows
# or
source venv/bin/activate      # Linux/Mac

# Test ATLAS NEXUS
python atlas_nexus.py
```

Try these commands:
```
You: hello
You: help
You: stats
You: Create a Python function to calculate fibonacci
```

---

## 🎮 Usage Modes

### 1. Interactive Mode (Default)
```bash
python atlas_nexus.py
```
Best for: Testing, learning, quick tasks

### 2. API Server Mode
```bash
python atlas_nexus.py api
```
Then open: http://localhost:8000/docs
Best for: Integration with other apps, mobile control

### 3. Telegram Bot Mode
```bash
python atlas_nexus.py telegram
```
Best for: Mobile usage, anywhere access

---

## 💡 First Tasks to Try

### Easy Tasks
```
Search the web for latest Python news
Create a hello world Python script
What's the weather like today?
List all files in C:\Users\
```

### Medium Tasks
```
Analyze this CSV file and create a summary
Search for top 5 AI companies and compare them
Create a Python class for handling user data
Read this PDF and extract key points
```

### Advanced Tasks
```
Create a web scraper for news websites
Build a data analysis pipeline for CSV files
Monitor this website and alert me of changes
Generate a complete project structure for a web app
```

---

## 🔧 Troubleshooting

### "Module not found" error
```bash
pip install -r requirements.txt
```

### "API key not configured"
Edit `C:\ATLAS\config\.env` and add your API keys

### "Ollama connection failed"
```bash
# Check if Ollama is running
ollama list

# Start Ollama (it should auto-start)
# If not, reinstall from https://ollama.ai
```

### "Port 8000 already in use"
```bash
# Change port in .env
ATLAS_API_PORT=8001

# Or kill the process using port 8000
# Windows:
netstat -ano | findstr :8000
taskkill /PID <process_id> /F
```

---

## 📱 Mobile Setup (Telegram)

1. **Create Bot**:
   - Message @BotFather on Telegram
   - Send `/newbot`
   - Follow instructions
   - Copy the bot token

2. **Get Your User ID**:
   - Message @userinfobot on Telegram
   - Copy your user ID

3. **Configure**:
   ```bash
   TELEGRAM_BOT_TOKEN=123456:ABC-DEF...
   TELEGRAM_OWNER_ID=123456789
   ```

4. **Start Bot**:
   ```bash
   python atlas_nexus.py telegram
   ```

5. **Test**:
   - Message your bot: `/start`
   - Try: "Search for AI news"

---

## 🎨 Dashboard Access

If API server is running:

1. Open browser
2. Go to: http://localhost:8000
3. Or view docs: http://localhost:8000/docs

---

## 📚 Next Steps

1. ✅ Complete this Quick Start
2. 📖 Read the full [README.md](README.md)
3. 🔧 Explore the [Tools](README.md#-available-tools)
4. 🚀 Try advanced features
5. 💬 Join the community (link TBD)

---

## 🆘 Getting Help

- 📖 Documentation: See README.md
- 🐛 Issues: GitHub Issues
- 💬 Community: Discord/Telegram (links TBD)
- 📧 Email: support@example.com

---

## 🎉 You're Ready!

ATLAS NEXUS is now configured and ready to use. Start with simple tasks and gradually explore more advanced features.

**Pro Tip**: Use `help` command in interactive mode to see all available commands!

---

<p align="center">
  <strong>Welcome to ATLAS NEXUS! 🚀</strong>
</p>
