# ATLAS Software Maintenance Report

- Start: `2026-03-06T02:30:18.937640+00:00`
- End: `2026-03-06T02:31:28.869792+00:00`
- Status: `done`
- Steps OK: `8`
- Steps Failed: `0`

## Steps
- `ensure_scheduler_jobs`: **OK**
- `makeplay_scan`: **OK**
- `ans_cycle`: **OK**
- `triada_run_once`: **OK**
- `repo_monitor_cycle`: **OK**
- `repo_update_cycle`: **OK**
- `refresh_tools_watchdog`: **OK**
  - Detail: `{"ok": true, "generated_at": "2026-03-06T02:31:08.374892+00:00", "workspace": "C:\\ATLAS_PUSH", "scan_source": "atlas_tools_watchdog.py", "summary": {"total": 18, "ok": 14, "warn": 3, "error": 1, "upg`
- `refresh_software_watchdog`: **OK**
  - Detail: `{"ok": true, "generated_at": "2026-03-06T02:31:28.741356+00:00", "workspace": "C:\\ATLAS_PUSH", "scan_source": "atlas_software_watchdog.py", "catalog_version": "1.0", "summary": {"software_total": 18,`

## Raw JSON
```json
{
  "ok": true,
  "started_at": "2026-03-06T02:30:18.937640+00:00",
  "finished_at": "2026-03-06T02:31:28.869792+00:00",
  "status": "done",
  "summary": {
    "total_steps": 8,
    "ok_steps": 8,
    "failed_steps": 0,
    "apply_repo": false
  },
  "steps": [
    {
      "step": "ensure_scheduler_jobs",
      "ok": true,
      "ms": 208,
      "details": {
        "ok": true,
        "ensured": [
          "ans_cycle",
          "makeplay_scanner",
          "repo_monitor_cycle"
        ]
      }
    },
    {
      "step": "makeplay_scan",
      "ok": true,
      "ms": 925,
      "details": {
        "ok": true,
        "snapshot": {
          "health_score": 100,
          "health_ok": true,
          "incidents_open": 0,
          "incidents_recent": [],
          "actions_last_hour": 0,
          "last_actions": [],
          "self_model_checks": 18,
          "self_model_heals": 11,
          "checks": {
            "api_up": true,
            "memory_writable": true,
            "audit_writable": true,
            "scheduler_alive": true,
            "llm_reachable": true,
            "avg_latency_ms": 0.0,
            "error_rate": 0.0,
            "ans_open_incidents": 0,
            "nervous_score": 100,
            "nervous_points_total": 0,
            "nervous_open_signals": 0,
            "nervous_by_severity_points": {
              "low": 0,
              "med": 0,
              "high": 0,
              "critical": 0
            },
            "active_port": 8791,
            "version": "0.0.0",
            "channel": "canary"
          },
          "webhook_pushed": false,
          "ts": "2026-03-06T02:30:20.072252+00:00"
        }
      }
    },
    {
      "step": "ans_cycle",
      "ok": true,
      "ms": 4873,
      "details": {
        "ok": true,
        "issues_count": 1,
        "actions_taken": 1,
        "report_path": "C:\\ATLAS_PUSH\\snapshots\\ans\\ANS_REPORT_20260306_023024.md"
      }
    },
    {
      "step": "triada_run_once",
      "ok": true,
      "ms": 26740,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "",
        "stderr_tail": "2026-03-05 21:30:25 [atlas.evolution] INFO Ejecutando un solo ciclo de la Tríada (PyPI | GitHub | HF)...\n2026-03-05 21:30:45 [atlas.evolution] INFO Tríada de Crecimiento: PyPI | GitHub | Hugging Face\n2026-03-05 21:30:48 [atlas.evolution] INFO [HF] robbyant/lingbot-vla-4b-depth (query=VLA) hash=e41a5ac6652d6655\n2026-03-05 21:30:48 [atlas.evolution] INFO [HF] robbyant/lingbot-vla-4b (query=VLA) hash=1b6b6416545add96\n2026-03-05 21:30:48 [atlas.evolution] INFO [HF] VladHong/Qwen3-4B-Instruct-NoThink-V2.1 (query=VLA) hash=785ccd6035c84351\n2026-03-05 21:30:48 [atlas.evolution] INFO [GitHub] mramirezraul71/atlas-core tag= commits=3 tools=[]\n2026-03-05 21:30:51 [atlas.evolution] INFO Ciclo completado.",
        "ms": 26740
      }
    },
    {
      "step": "repo_monitor_cycle",
      "ok": true,
      "ms": 6746,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "",
        "stderr_tail": "",
        "ms": 6746
      }
    },
    {
      "step": "repo_update_cycle",
      "ok": true,
      "ms": 829,
      "details": {
        "ok": true,
        "phase": "check",
        "data": {
          "ok": true,
          "enabled": true,
          "branch": "dev",
          "head_commit": "553464fb17cb5b404fa1d4d2ef80c1e841e863e9",
          "remote_commit": "eb6525bdfea7be7765ddc97276af156cced81909",
          "has_update": true,
          "config": {
            "remote": "origin",
            "branch": "main",
            "staging_branch": "staging",
            "require_smoke": true,
            "auto_promote": false,
            "allow_rollback": true,
            "use_staging": false,
            "update_window_start": "01:00",
            "update_window_end": "04:00",
            "in_update_window": true
          },
          "ms": 490,
          "error": null,
          "snapshot": {
            "status_output": "m _external/rauli-panaderia\n M atlas_adapter/atlas_http_api.py\n M atlas_adapter/static/v4/modules/apps.js\n M config/atlas.env.example\n M contracts/atlas_comms_messages.schema.sql\n M modules/humanoid/comms/atlas_comms_hub.py\n M modules/humanoid/comms/hub.py\n M modules/humanoid/comms/telegram_bridge.py\n M modules/humanoid/comms/telegram_poller.py\n M modules/humanoid/comms/whatsapp_bridge.py",
            "diff_output": "diff --git a/_external/rauli-panaderia b/_external/rauli-panaderia\n--- a/_external/rauli-panaderia\n+++ b/_external/rauli-panaderia\n@@ -1 +1 @@\n-Subproject commit 6a99384b3a21f0657372c0fff789fbdd54474ed8\n+Subproject commit 6a99384b3a21f0657372c0fff789fbdd54474ed8-dirty\ndiff --git a/atlas_adapter/atlas_http_api.py b/atlas_adapter/atlas_http_api.py\nindex e188019ef..5cf8e4b89 100644\n--- a/atlas_adapter/atlas_http_api.py\n+++ b/atlas_adapter/atlas_http_api.py\n@@ -5,6 +5,7 @@ import asyncio\n import json\n import logging\n import os\n+import re\n import sys\n import tempfile\n import threading\n@@ -3025,8 +3026,8 @@ def api_whatsapp_status():\n         return {\"ok\": False, \"error\": str(e)}\n \n \n-@app.post(\"/api/comms/whatsapp/send\", tags=[\"Comms\", \"WhatsApp\"])\n-def api_whatsapp_send(body: dict):\n+@app.post(\"/api/comms/whatsapp/send\", tags=[\"Comms\", \"WhatsApp\"])\n+def api_whatsapp_send(body: dict):\n     \"\"\"EnvÃ­a un mensaje de WhatsApp.\n \n     Body: {\"text\": \"mensaje\", \"to\": \"+34612345678\" (opcional)}\n@@ -3040,13 +3041,80 @@ def api_whatsapp_send(body: dict):\n         if not text:\n             return {\"ok\": False, \"error\": \"text is required\"}\n \n-        return send_text(text, to=to)\n-    except Exception as e:\n-        return {\"ok\": False, \"error\": str(e)}\n-\n-\n-@app.post(\"/api/comms/whatsapp/test\", tags=[\"Comms\", \"WhatsApp\"])\n-def api_whatsapp_test():\n+        return send_text(text, to=to)\n+    except Exception as e:\n+        return {\"ok\": False, \"error\": str(e)}\n+\n+\n+@app.post(\"/api/comms/whatsapp/inbound\", tags=[\"Comms\", \"WhatsApp\"])\n+def api_whatsapp_inbound(body: dict):\n+    \"\"\"\n+    Webhook inbound de WhatsApp (WAHA/Twilio/Meta) -> supervisor conversacional.\n+\n+    Flujo:\n+    inbound payload -> parse -> atlas_comms_hub.process_user_interaction -> send_text al remitente\n+    \"\"\"\n+    try:\n+        from modules.humanoid.comms.atlas_comms_hub import get_atlas_comms_hub\n+        from modules.humanoid.comms.whatsapp_bridge import (parse_inbound_payload,\n+                                                            send_text)\n+\n+        parsed = parse_inbound_payload(body or {})\n+        if not parsed.get(\"ok\"):\n+            return {\n+                \"ok\": True,\n+                \"ignored\": True,\n+                \"reason\": parsed.get(\"reason\") or \"unparsed_payload\",\n+                \"provider_hint\": parsed.get(\"provider_hint\"),\n+            }\n+\n+        sender = str(parsed.get(\"from\") or \"\").strip()\n+        text = str(parsed.get(\"text\") or \"\").strip()\n+        if not sender or not text:\n+            return {\"ok\": True, \"ignored\": True, \"reason\": \"missing_sender_or_text\"}\n+\n+        hub = get_atlas_comms_hub()\n+        resolved_user = hub.resolve_external_user_by_whatsapp(sender)\n+        effective_user_id = (\n+            str(resolved_user.get(\"user_id\"))\n+            if resolved_user.get(\"ok\")\n+            else str(parsed.get(\"user_id\") or f\"whatsapp:{sender}\")\n+        )\n+        out = hub.process_user_interaction(\n+            user_id=effective_user_id,\n+            channel=\"whatsapp\",\n+            message=text,\n+            context={\n+                \"source\": \"whatsapp_inbound\",\n+                \"provider_hint\": parsed.get(\"provider_hint\"),\n+                \"sender\": sender,\n+                \"external_user\": resolved_user if resolved_user.get(\"ok\") else None,\n+            },\n+        )\n+\n+        reply_text = str(out.get(\"reply\") or \"\").strip()\n+        send_result = {\"ok\": False, \"skipped\": True}\n+        if reply_text:\n+            send_result = send_text(reply_text, to=sender)\n+\n+        return {\n+            \"ok\": bool(out.get(\"ok\")),\n+            \"inbound\": parsed,\n+            \"interaction\": {\n+                \"message_id\": out.get(\"message_id\"),\n+                \"provider\": out.get(\"provider\"),\n+                \"offline_mode\": out.get(\"offline_mode\"),\n+                \"urgency\": out.get(\"urgency\"),\n+                \"resolved_user\": resolved_user if resolved_user.get(\"ok\") else None,\n+            },\n+            \"send_result\": send_result,\n+        }\n+    except Exception as e:\n+        return {\"ok\": False, \"error\": str(e)}\n+\n+\n+"
          }
        }
      }
    },
    {
      "step": "refresh_tools_watchdog",
      "ok": true,
      "ms": 9127,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "{\"ok\": true, \"generated_at\": \"2026-03-06T02:31:08.374892+00:00\", \"workspace\": \"C:\\\\ATLAS_PUSH\", \"scan_source\": \"atlas_tools_watchdog.py\", \"summary\": {\"total\": 18, \"ok\": 14, \"warn\": 3, \"error\": 1, \"upgrade_ready\": 2}, \"offline_queue\": {\"pending\": 0, \"db_path\": \"C:\\\\ATLAS_PUSH\\\\logs\\\\atlas_comms_hub.sqlite\", \"resync_script\": \"scripts\\\\atlas_resync_flow.ps1\"}, \"channels\": {\"panaderia\": \"http://127.0.0.1:3001\", \"vision\": \"http://127.0.0.1:3000\", \"push\": \"http://127.0.0.1:8791\"}, \"git\": {\"branch\": \"dev\", \"is_protected\": false, \"protected_branches\": [\"main\", \"master\", \"prod\", \"production\", \"release\"], \"allow_protected_updates\": false}, \"security\": {\"core_token_env\": false, \"approval_secret_env\": true, \"encryption_mode\": \"APPROVALS_CHAIN_SECRET\"}, \"tools\": [{\"id\": \"jq\", \"name\": \"jq\", \"category\": \"data\", \"critical\": false, \"version\": \"1.8.1\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"jq-1.8.1\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"ccxt\", \"name\": \"CCXT (Trading)\", \"category\": \"dependency\", \"critical\": false, \"version\": \"4.5.41\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool ccxt\", \"latest_version\": \"4.5.41\", \"update_ready\": false}, {\"id\": \"playwright_py\", \"name\": \"Playwright Python\", \"category\": \"dependency\", \"critical\": false, \"version\": \"1.58.0\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool playwright_py\", \"latest_version\": \"1.58.0\", \"update_ready\": false}, {\"id\": \"puppeteer\", \"name\": \"Puppeteer Node\", \"category\": \"dependency\", \"critical\": false, \"version\": \"24.38.0\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"tools/atlas_actuators\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool puppeteer\", \"latest_version\": \"24.38.0\", \"update_ready\": false}, {\"id\": \"git\", \"name\": \"Git CLI\", \"category\": \"devops\", \"critical\": true, \"version\": \"2.53.0.windows.1\", \"status\": \"ok\", \"health\": \"online\", \"details\": \"git version 2.53.0.windows.1\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool git\", \"latest_version\": \"2.53.0.windows.1\", \"update_ready\": false}, {\"id\": \"yt-dlp\", \"name\": \"yt-dlp\", \"category\": \"media\", \"critical\": false, \"version\": \"2026.3.3\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"venv_package=2026.3.3\", \"update_script\": \"\", \"latest_version\": \"2026.3.3\", \"update_ready\": false}, {\"id\": \"cloudflared\", \"name\": \"Cloudflare Tunnel\", \"category\": \"network\", \"critical\": true, \"version\": \"2025.11.1\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"cloudflared version 2025.11.1 (built 2025-11-07T16:30 UTC)\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool cloudflared\", \"latest_version\": \"2026.2.0\", \"update_ready\": true}, {\"id\": \"httpie\", \"name\": \"HTTPie\", \"category\": \"network\", \"critical\": false, \"version\": \"2025.2.0\", \"status\": \"warn\", \"health\": \"installed_no_path\", \"details\": \"installed via winget (PATH pendiente)\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"nmap\", \"name\": \"Nmap\", \"category\": \"network\", \"critical\": false, \"version\": \"5.3.5\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Nmap version 7.80 ( https://nmap.org )\\nPlatform: i686-pc-windows-windows\\nCompiled with: nmap-liblua-5.3.5 openssl-1.0.2s nmap-libssh2-1.8.2 nmap-libz-1.2.11 nmap-libpcre-7.6 Npcap-0.9982 nmap-libdnet-1.12 ipv6\\nCompiled without:\\nAvailable nsock engines: iocp poll select\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"tailscale\", \"name\": \"Tailscale Backup Tunnel\", \"category\": \"network\", \"critical\": true, \"version\": \"\", \"status\": \"error\", \"health\": \"not_detected\", \"details\": \"[WinError 2] El sistema no puede encontrar el archivo especificado\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool tailscale\", \"latest_version\": \"1.94.2\", \"update_ready\": false}, {\"id\": \"task\", \"name\": \"Taskwarrior\", \"category\": \"orchestration\", \"critical\": false, \"version\": \"3.48.0\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"3.48.0\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"ruff\", \"name\": \"ruff\", \"category\": \"quality\", \"critical\": false, \"version\": \"0.15.5\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"venv_package=0.15.5; cmd=ruff 0.15.2\", \"update_script\": \"\", \"latest_version\": \"0.15.5\", \"update_ready\": false}, {\"id\": \"sqlite\", \"name\": \"SQLite Local Store\", \"category\": \"resilience\", \"critical\": false, \"version\": \"3.45.1\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"offline_queue_pending=0\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"node\", \"name\": \"Node.js\", \"category\": \"runtime\", \"critical\": true, \"version\": \"25.8.0\", \"status\": \"ok\", \"health\": \"online\", \"details\": \"v25.8.0\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool node\", \"latest_version\": \"25.8.0\", \"update_ready\": false}, {\"id\": \"ollama\", \"name\": \"Ollama Runtime\", \"category\": \"runtime\", \"critical\": true, \"version\": \"0.17.6\", \"status\": \"ok\", \"health\": \"online\", \"details\": \"\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool ollama\", \"latest_version\": \"0.17.6\", \"update_ready\": false}, {\"id\": \"uv\", \"name\": \"uv\", \"category\": \"runtime\", \"critical\": false, \"version\": \"0.7.20\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"uv 0.7.20 (251420396 2025-07-09)\", \"update_script\": \"\", \"latest_version\": \"0.10.8\", \"update_ready\": true}, {\"id\": \"clawdbot_core\", \"name\": \"ClawdBOT Core Token\", \"category\": \"security\", \"critical\": true, \"version\": \"configured\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"ATLAS_CENTRAL_CORE / APPROVALS_CHAIN_SECRET\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"tmux\", \"name\": \"tmux\", \"category\": \"terminal\", \"critical\": false, \"version\": \"3.6\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"tmux 3.6a-win32\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}]}",
        "stderr_tail": "",
        "ms": 9127
      }
    },
    {
      "step": "refresh_software_watchdog",
      "ok": true,
      "ms": 20467,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "{\"ok\": true, \"generated_at\": \"2026-03-06T02:31:28.741356+00:00\", \"workspace\": \"C:\\\\ATLAS_PUSH\", \"scan_source\": \"atlas_software_watchdog.py\", \"catalog_version\": \"1.0\", \"summary\": {\"software_total\": 18, \"software_ok\": 9, \"software_warn\": 9, \"software_error\": 0, \"software_update_ready\": 5, \"candidates_total\": 2, \"candidates_available\": 2, \"drivers_total\": 4, \"drivers_ok\": 4, \"drivers_warn\": 0, \"drivers_error\": 0, \"dev_total\": 9, \"dev_ok\": 5, \"dev_warn\": 1, \"dev_error\": 3}, \"software\": [{\"id\": \"docker\", \"name\": \"Docker Desktop\", \"category\": \"containers\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"Docker.DockerDesktop\", \"installed\": true, \"version\": \"29.0.1\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Docker version 29.0.1, build eedd969\", \"network_probe\": \"https://docs.docker.com/desktop/\", \"network_available\": true, \"latency_ms\": 198, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"jq\", \"name\": \"jq\", \"category\": \"data\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"jqlang.jq\", \"installed\": true, \"version\": \"1.8.1\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"jq-1.8.1\", \"network_probe\": \"https://jqlang.github.io/jq/\", \"network_available\": true, \"latency_ms\": 212, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"playwright_py\", \"name\": \"Playwright Python\", \"category\": \"dependency\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"playwright\", \"installed\": true, \"version\": \"1.46.0\", \"latest_version\": \"1.58.0\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"Version 1.46.0\", \"network_probe\": \"https://pypi.org/pypi/playwright/json\", \"network_available\": true, \"latency_ms\": 97, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"git\", \"name\": \"Git CLI\", \"category\": \"devops\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Git.Git\", \"installed\": true, \"version\": \"2.53.0.windows.1\", \"latest_version\": \"2.53.0.windows.1\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"git version 2.53.0.windows.1\", \"network_probe\": \"https://api.github.com/repos/git-for-windows/git/releases/latest\", \"network_available\": true, \"latency_ms\": 187, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"cloudflared\", \"name\": \"Cloudflare Tunnel\", \"category\": \"network\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Cloudflare.cloudflared\", \"installed\": true, \"version\": \"2025.11.1\", \"latest_version\": \"2026.2.0\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"cloudflared version 2025.11.1 (built 2025-11-07T16:30 UTC)\", \"network_probe\": \"https://api.github.com/repos/cloudflare/cloudflared/releases/latest\", \"network_available\": true, \"latency_ms\": 184, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"httpie\", \"name\": \"HTTPie\", \"category\": \"network\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"HTTPie.HTTPie\", \"installed\": true, \"version\": \"2025.2.0\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"installed_no_path\", \"details\": \"- \\n                                                                                                                        \\n\\n   - \\n   \\\\ \\n   | \\n                                                                                                                        \\nNombre          Id            VersiÃ³n\\n--------------------------------------\\nHTTPie 2025.2.0 HTTPie.HTTPie 2025.2.0\", \"network_probe\": \"https://httpie.io\", \"network_available\": true, \"latency_ms\": 220, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"nmap\", \"name\": \"Nmap\", \"category\": \"network\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"Insecure.Nmap\", \"installed\": true, \"version\": \"5.3.5\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Nmap version 7.80 ( https://nmap.org )\\nPlatform: i686-pc-windows-windows\\nCompiled with: nmap-liblua-5.3.5 openssl-1.0.2s nmap-libssh2-1.8.2 nmap-libz-1.2.11 nmap-libpcre-7.6 Npcap-0.9982 nmap-libdnet-1.12 ipv6\\nCompiled without:\\nAvailable nsock engines: iocp poll select\", \"network_probe\": \"https://nmap.org\", \"network_available\": true, \"latency_ms\": 345, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"tailscale\", \"name\": \"Tailscale\", \"category\": \"network\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Tailscale.Tailscale\", \"installed\": true, \"version\": \"1.94.2\", \"latest_version\": \"1.94.2\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"installed_no_path\", \"details\": \"- \\n                                                                                                                        \\n\\n   - \\n   \\\\ \\n   | \\n                                                                                                                        \\nNombre    Id                  VersiÃ³n\\n-------------------------------------\\nTailscale Tailscale.Tailscale 1.94.2\", \"network_probe\": \"https://api.github.com/repos/tailscale/tailscale/releases/latest\", \"network_available\": true, \"latency_ms\": 164, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"taskwarrior\", \"name\": \"Taskwarrior\", \"category\": \"orchestration\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"Task.Task\", \"installed\": true, \"version\": \"3.48.0\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"3.48.0\", \"network_probe\": \"https://taskwarrior.org\", \"network_available\": true, \"latency_ms\": 513, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ruff\", \"name\": \"ruff\", \"category\": \"quality\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"ruff\", \"installed\": true, \"version\": \"0.15.2\", \"latest_version\": \"0.15.5\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"ruff 0.15.2\", \"network_probe\": \"https://pypi.org/pypi/ruff/json\", \"network_available\": true, \"latency_ms\": 107, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"node\", \"name\": \"Node.js\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"OpenJS.NodeJS\", \"installed\": true, \"version\": \"25.8.0\", \"latest_version\": \"25.8.0\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"v25.8.0\", \"network_probe\": \"https://nodejs.org/dist/index.json\", \"network_available\": true, \"latency_ms\": 140, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ollama\", \"name\": \"Ollama\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Ollama.Ollama\", \"installed\": true, \"version\": \"0.17.6\", \"latest_version\": \"0.17.6\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"ollama version is 0.17.6\", \"network_probe\": \"https://api.github.com/repos/ollama/ollama/releases/latest\", \"network_available\": true, \"latency_ms\": 179, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"python\", \"name\": \"Python Runtime\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Python.Python.3.12\", \"installed\": true, \"version\": \"3.11.9\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Python 3.11.9\", \"network_probe\": \"https://pypi.org/pypi/pip/json\", \"network_available\": true, \"latency_ms\": 91, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"pip\", \"name\": \"pip\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"pip\", \"install_target\": \"pip\", \"installed\": true, \"version\": \"25.3\", \"latest_version\": \"26.0.1\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"pip 25.3 from C:\\\\Users\\\\r6957\\\\AppData\\\\Local\\\\Programs\\\\Python\\\\Python311\\\\Lib\\\\site-packages\\\\pip (python 3.11)\", \"network_probe\": \"https://pypi.org/pypi/pip/json\", \"network_available\": true, \"latency_ms\": 134, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"uv\", \"name\": \"uv\", \"category\": \"runtime\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"astral-sh.uv\", \"installed\": true, \"version\": \"0.7.20\", \"latest_version\": \"0.10.8\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"uv 0.7.20 (251420396 2025-07-09)\", \"network_probe\": \"https://pypi.org/pypi/uv/json\", \"network_available\": true, \"latency_ms\": 123, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"tmux\", \"name\": \"tmux\", \"category\": \"terminal\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"arndawg.tmux-windows\", \"installed\": true, \"version\": \"3.6\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"tmux 3.6a-win32\", \"network_probe\": \"https://github.com/tmux/tmux\", \"network_available\": true, \"latency_ms\": 363, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ccxt\", \"name\": \"CCXT\", \"category\": \"dependency\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"ccxt\", \"installed\": false, \"version\": \"\", \"latest_version\": \"4.5.41\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"not_detected\", \"details\": \"\", \"network_probe\": \"https://pypi.org/pypi/ccxt/json\", \"network_available\": true, \"latency_ms\": 122, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ytdlp\", \"name\": \"yt-dlp\", \"category\": \"media\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"yt-dlp\", \"installed\": false, \"version\": \"\", \"latest_version\": \"2026.3.3\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"not_detected\", \"details\": \"\", \"network_probe\": \"https://pypi.org/pypi/yt-dlp/json\", \"network_available\": true, \"latency_ms\": 108, \"network_status\": \"network_ok\", \"network_error\": null}], \"network_candidates\": [{\"id\": \"ccxt\", \"name\": \"CCXT\", \"category\": \"dependency\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"ccxt\", \"network_available\": true, \"latency_ms\": 122, \"network_error\": null, \"latest_version\": \"4.5.41\", \"score\": 90, \"status\": \"ready\"}, {\"id\": \"ytdlp\", \"name\": \"yt-dlp\", \"category\": \"media\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"yt-dlp\", \"network_available\": true, \"latency_ms\": 108, \"network_error\": null, \"latest_version\": \"2026.3.3\", \"score\": 90, \"status\": \"ready\"}], \"drivers\": [{\"id\": \"audio_driver\", \"name\": \"Audio Driver\", \"critical\": false, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"10.0.26100.1\", \"matches\": [{\"name\": \"Microsoft Streaming Service Proxy\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"MEDIA\"}, {\"name\": \"NVIDIA Virtual Audio Device (Wave Extensible) (WDM)\", \"version\": \"4.65.0.3\", \"provider\": \"NVIDIA\", \"class\": \"MEDIA\"}, {\"name\": \"Realtek High Definition Audio\", \"version\": \"6.0.9549.1\", \"provider\": \"Realtek Semiconductor Corp.\", \"class\": \"MEDIA\"}, {\"name\": \"Microsoft Bluetooth Hands-Free Audio device\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"MEDIA\"}, {\"name\": \"Microsoft Bluetooth Hands-Free Audio device\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"MEDIA\"}], \"count\": 10}, {\"id\": \"bluetooth_driver\", \"name\": \"Bluetooth Driver\", \"critical\": false, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"10.0.26100.7920\", \"matches\": [{\"name\": \"Bluetooth LE Generic Attribute Service\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Device Information Service\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Generic Attribute Profile\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Generic Access Profile\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Bluetooth LE Device\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}], \"count\": 20}, {\"id\": \"gpu_display_driver\", \"name\": \"GPU Display Driver\", \"critical\": true, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"32.0.15.9155\", \"matches\": [{\"name\": \"NVIDIA GeForce GTX 1660 SUPER\", \"version\": \"32.0.15.9155\", \"date\": \"/Date(1765238400000)/\"}], \"count\": 1}, {\"id\": \"network_adapter_driver\", \"name\": \"Network Adapter Driver\", \"critical\": true, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"0.14.0.0\", \"matches\": [{\"name\": \"Tailscale Tunnel\", \"version\": \"0.14.0.0\", \"provider\": \"WireGuard LLC\", \"class\": \"NET\"}, {\"name\": \"Microsoft KM-TEST Loopback Adapter\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"Hyper-V Virtual Switch Extension Adapter\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"WAN Miniport (Network Monitor)\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"WAN Miniport (IPv6)\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}], \"count\": 18}], \"development_stack\": [{\"id\": \"ans_scheduler\", \"name\": \"ANS Scheduler Job\", \"type\": \"scheduler_job\", \"critical\": true, \"status\": \"ok\", \"health\": \"success\", \"version\": \"\", \"details\": \"job=ans_cycle\"}, {\"id\": \"nexus_api\", \"name\": \"ATLAS NEXUS API\", \"type\": \"service\", \"critical\": false, \"status\": \"ok\", \"health\": \"online\", \"version\": \"2.0.0\", \"details\": \"\"}, {\"id\": \"push_api\", \"name\": \"ATLAS PUSH API\", \"type\": \"service\", \"critical\": true, \"status\": \"ok\", \"health\": \"online\", \"version\": \"unknown\", \"details\": \"\"}, {\"id\": \"robot_api\", \"name\": \"ATLAS Robot API\", \"type\": \"service\", \"critical\": false, \"status\": \"ok\", \"health\": \"online\", \"version\": \"\", \"details\": \"\"}, {\"id\": \"triada_last_cycle\", \"name\": \"Triada Last Cycle\", \"type\": \"log_file\", \"critical\": true, \"status\": \"ok\", \"health\": \"present\", \"version\": \"2026-03-06T02:30:51.624059+00:00\", \"details\": \"Asimilación Exitosa\"}, {\"id\": \"makeplay_last_scan\", \"name\": \"MakePlay Last Scan\", \"type\": \"makeplay_last_scan\", \"critical\": true, \"status\": \"error\", \"health\": \"empty\", \"version\": \"\", \"details\": \"no_makeplay_scan_data\"}, {\"id\": \"makeplay_scheduler\", \"name\": \"MakePlay Scheduler Job\", \"type\": \"scheduler_job\", \"critical\": true, \"status\": \"error\", \"health\": \"failed\", \"version\": \"\", \"details\": \"job=makeplay_scanner\"}, {\"id\": \"repo_monitor_scheduler\", \"name\": \"Repo Monitor Scheduler Job\", \"type\": \"scheduler_job\", \"critical\": true, \"status\": \"error\", \"health\": \"paused\", \"version\": \"\", \"details\": \"job=repo_monitor_cycle\"}, {\"id\": \"repo_update_status\", \"name\": \"Repo Update Status\", \"type\": \"repo_update\", \"critical\": true, \"status\": \"warn\", \"health\": \"update_ready\", \"version\": \"dev\", \"details\": \"head=553464fb remote=eb6525bd\"}]}",
        "stderr_tail": "",
        "ms": 20467
      }
    }
  ]
}
```