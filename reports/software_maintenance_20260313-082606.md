# ATLAS Software Maintenance Report

- Start: `2026-03-13T12:24:26.025656+00:00`
- End: `2026-03-13T12:26:06.806269+00:00`
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
  - Detail: `{"ok": true, "generated_at": "2026-03-13T12:25:25.025188+00:00", "workspace": "C:\\ATLAS_PUSH", "scan_source": "atlas_tools_watchdog.py", "summary": {"total": 18, "ok": 9, "warn": 9, "error": 0, "upgr`
- `refresh_software_watchdog`: **OK**
  - Detail: `{"ok": true, "generated_at": "2026-03-13T12:26:06.687450+00:00", "workspace": "C:\\ATLAS_PUSH", "scan_source": "atlas_software_watchdog.py", "catalog_version": "1.0", "summary": {"software_total": 18,`

## Raw JSON
```json
{
  "ok": true,
  "started_at": "2026-03-13T12:24:26.025656+00:00",
  "finished_at": "2026-03-13T12:26:06.806269+00:00",
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
      "ms": 535,
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
      "ms": 10698,
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
            "version": "1.0.0-alpha-764-g70498b6ec-dirty",
            "channel": "canary"
          },
          "webhook_pushed": false,
          "ts": "2026-03-13T12:24:37.258697+00:00"
        }
      }
    },
    {
      "step": "ans_cycle",
      "ok": true,
      "ms": 5505,
      "details": {
        "ok": true,
        "issues_count": 2,
        "actions_taken": 1,
        "report_path": "C:\\ATLAS_PUSH\\snapshots\\ans\\ANS_REPORT_20260313_122442.md"
      }
    },
    {
      "step": "triada_run_once",
      "ok": true,
      "ms": 30603,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "",
        "stderr_tail": "2026-03-13 08:24:42 [atlas.evolution] INFO Ejecutando un solo ciclo de la Tríada (PyPI | GitHub | HF)...\n2026-03-13 08:24:53 [atlas.evolution] INFO Tríada de Crecimiento: PyPI | GitHub | Hugging Face\n2026-03-13 08:24:56 [atlas.evolution] INFO [HF] robbyant/lingbot-vla-4b-posttrain-robotwin (query=VLA) hash=e5aff53e5bca8e33\n2026-03-13 08:24:56 [atlas.evolution] INFO [GitHub] mramirezraul71/atlas-core tag= commits=3 tools=[]\n2026-03-13 08:24:56 [atlas.evolution] INFO [HF] robbyant/lingbot-vla-4b-depth-posttrain-robotwin (query=VLA) hash=2ce2532a3991ef42\n2026-03-13 08:24:56 [atlas.evolution] INFO [HF] VladKobranov/splats (query=VLA) hash=4bbd992e6d14814f\n2026-03-13 08:25:13 [atlas.evolution] INFO Ciclo completado.",
        "ms": 30603
      }
    },
    {
      "step": "repo_monitor_cycle",
      "ok": true,
      "ms": 680,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "",
        "stderr_tail": "",
        "ms": 680
      }
    },
    {
      "step": "repo_update_cycle",
      "ok": true,
      "ms": 807,
      "details": {
        "ok": true,
        "phase": "check",
        "data": {
          "ok": true,
          "enabled": true,
          "branch": "chore/cloudflare-autonomous-fallback",
          "head_commit": "70498b6ec3ee8971981756cb6bc29c49cdaeeee0",
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
            "in_update_window": false
          },
          "ms": 461,
          "error": null,
          "snapshot": {
            "status_output": "M _external/RAULI-VISION\n M atlas_adapter/atlas_http_api.py\n?? modules/humanoid/avatar/",
            "diff_output": "diff --git a/_external/RAULI-VISION b/_external/RAULI-VISION\nindex d4674702b..055e26754 160000\n--- a/_external/RAULI-VISION\n+++ b/_external/RAULI-VISION\n@@ -1 +1 @@\n-Subproject commit d4674702b7787a84a3b5b69fa1f8a23eb53eb5e4\n+Subproject commit 055e2675440de4b806a56db16b017e67ab52f4d8-dirty\ndiff --git a/atlas_adapter/atlas_http_api.py b/atlas_adapter/atlas_http_api.py\nindex bf2aa875f..09032da01 100644\n--- a/atlas_adapter/atlas_http_api.py\n+++ b/atlas_adapter/atlas_http_api.py\n@@ -4078,7 +4078,7 @@ def _call_claude_cli(prompt: str, system: str, timeout_s: int) -> tuple[bool, st\n     return True, out, \"clawd_cli\"\n \n \n-class AtlasClawdSubscriptionBody(BaseModel):\n+class AtlasClawdSubscriptionBody(BaseModel):\n     message: str\n     context: Dict[str, Any] = Field(default_factory=dict)\n     persona: str = \"friendly_precise_assistant\"\n@@ -4088,25 +4088,62 @@ class AtlasClawdSubscriptionBody(BaseModel):\n     @classmethod\n     def validate_message(cls, value: str) -> str:\n         msg = (value or \"\").strip()\n-        if not msg:\n-            raise ValueError(\"message cannot be empty\")\n-        return msg\n-\n-\n-@app.get(\"/api/comms/atlas/status\", tags=[\"Comms\"])\n-def api_comms_atlas_status():\n+        if not msg:\n+            raise ValueError(\"message cannot be empty\")\n+        return msg\n+\n+\n+class AvatarRunwayLineBody(BaseModel):\n+    topic: str = \"\"\n+    source: str = \"api\"\n+    requested_line: str = \"\"\n+\n+    @field_validator(\"topic\", \"source\", \"requested_line\")\n+    @classmethod\n+    def _trim_fields(cls, value: str) -> str:\n+        return (value or \"\").strip()\n+\n+\n+@app.get(\"/api/comms/atlas/status\", tags=[\"Comms\"])\n+def api_comms_atlas_status():\n     \"\"\"ATLAS comms bridge status (offline mode, queue, encryption source).\"\"\"\n     try:\n         from modules.humanoid.comms.atlas_comms_hub import get_atlas_comms_hub\n \n         hub = get_atlas_comms_hub()\n         return hub.get_status()\n-    except Exception as e:\n-        return {\"ok\": False, \"error\": str(e)}\n-\n-\n-@app.get(\"/api/comms/atlas/history\", tags=[\"Comms\"])\n-def api_comms_atlas_history(limit: int = 30):\n+    except Exception as e:\n+        return {\"ok\": False, \"error\": str(e)}\n+\n+\n+@app.get(\"/api/avatar/runway/context\", tags=[\"Avatar\", \"Runway\"])\n+def api_avatar_runway_context():\n+    \"\"\"Runtime payload to drive ATLAS avatar behavior in Runway Characters.\"\"\"\n+    try:\n+        from modules.humanoid.avatar import build_runway_character_context\n+\n+        return build_runway_character_context()\n+    except Exception as e:\n+        return {\"ok\": False, \"error\": str(e)}\n+\n+\n+@app.post(\"/api/avatar/runway/line\", tags=[\"Avatar\", \"Runway\"])\n+def api_avatar_runway_line(body: AvatarRunwayLineBody):\n+    \"\"\"Low-latency short line payload for Runway Characters scene driving.\"\"\"\n+    try:\n+        from modules.humanoid.avatar import build_runway_line\n+\n+        return build_runway_line(\n+            topic=body.topic,\n+            source=body.source,\n+            requested_line=body.requested_line,\n+        )\n+    except Exception as e:\n+        return {\"ok\": False, \"error\": str(e)}\n+\n+\n+@app.get(\"/api/comms/atlas/history\", tags=[\"Comms\"])\n+def api_comms_atlas_history(limit: int = 30):\n     \"\"\"Recent user interactions processed by atlas_comms_hub.\"\"\"\n     try:\n         from modules.humanoid.comms.atlas_comms_hub import get_atlas_comms_hub\n"
          }
        }
      }
    },
    {
      "step": "refresh_tools_watchdog",
      "ok": true,
      "ms": 10177,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "{\"ok\": true, \"generated_at\": \"2026-03-13T12:25:25.025188+00:00\", \"workspace\": \"C:\\\\ATLAS_PUSH\", \"scan_source\": \"atlas_tools_watchdog.py\", \"summary\": {\"total\": 18, \"ok\": 9, \"warn\": 9, \"error\": 0, \"upgrade_ready\": 8}, \"offline_queue\": {\"pending\": 0, \"db_path\": \"C:\\\\ATLAS_PUSH\\\\logs\\\\atlas_comms_hub.sqlite\", \"resync_script\": \"scripts\\\\atlas_resync_flow.ps1\"}, \"channels\": {\"panaderia\": \"http://127.0.0.1:3001\", \"vision\": \"http://127.0.0.1:3000\", \"push\": \"http://127.0.0.1:8791\"}, \"git\": {\"branch\": \"chore/cloudflare-autonomous-fallback\", \"is_protected\": false, \"protected_branches\": [\"main\", \"master\", \"prod\", \"production\", \"release\"], \"allow_protected_updates\": false}, \"security\": {\"core_token_env\": true, \"approval_secret_env\": true, \"encryption_mode\": \"ATLAS_CENTRAL_CORE\"}, \"tools\": [{\"id\": \"jq\", \"name\": \"jq\", \"category\": \"data\", \"critical\": false, \"version\": \"1.8.1\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"jq-1.8.1\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool jq\", \"latest_version\": \"1.8.1\", \"update_ready\": false}, {\"id\": \"ccxt\", \"name\": \"CCXT (Trading)\", \"category\": \"dependency\", \"critical\": false, \"version\": \"4.5.42\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool ccxt\", \"latest_version\": \"4.5.43\", \"update_ready\": true}, {\"id\": \"playwright_py\", \"name\": \"Playwright Python\", \"category\": \"dependency\", \"critical\": false, \"version\": \"1.58.0\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool playwright_py\", \"latest_version\": \"1.58.0\", \"update_ready\": false}, {\"id\": \"puppeteer\", \"name\": \"Puppeteer Node\", \"category\": \"dependency\", \"critical\": false, \"version\": \"24.38.0\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"tools/atlas_actuators\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool puppeteer\", \"latest_version\": \"24.39.1\", \"update_ready\": true}, {\"id\": \"git\", \"name\": \"Git CLI\", \"category\": \"devops\", \"critical\": true, \"version\": \"2.53.0.windows.1\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"git version 2.53.0.windows.1\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool git\", \"latest_version\": \"2.53.0.windows.2\", \"update_ready\": true}, {\"id\": \"yt-dlp\", \"name\": \"yt-dlp\", \"category\": \"media\", \"critical\": false, \"version\": \"2026.3.3\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"venv_package=2026.3.3\", \"update_script\": \"\", \"latest_version\": \"2026.3.13\", \"update_ready\": true}, {\"id\": \"cloudflared\", \"name\": \"Cloudflare Tunnel\", \"category\": \"network\", \"critical\": true, \"version\": \"2025.11.1\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"cloudflared version 2025.11.1 (built 2025-11-07T16:30 UTC)\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool cloudflared\", \"latest_version\": \"2026.3.0\", \"update_ready\": true}, {\"id\": \"httpie\", \"name\": \"HTTPie\", \"category\": \"network\", \"critical\": false, \"version\": \"2025.2.0\", \"status\": \"warn\", \"health\": \"installed_no_path\", \"details\": \"installed via winget (PATH pendiente)\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool httpie\", \"latest_version\": \"3.2.4\", \"update_ready\": false}, {\"id\": \"nmap\", \"name\": \"Nmap\", \"category\": \"network\", \"critical\": false, \"version\": \"5.3.5\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Nmap version 7.80 ( https://nmap.org )\\nPlatform: i686-pc-windows-windows\\nCompiled with: nmap-liblua-5.3.5 openssl-1.0.2s nmap-libssh2-1.8.2 nmap-libz-1.2.11 nmap-libpcre-7.6 Npcap-0.9982 nmap-libdnet-1.12 ipv6\\nCompiled without:\\nAvailable nsock engines: iocp poll select\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool nmap\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"tailscale\", \"name\": \"Tailscale Backup Tunnel\", \"category\": \"network\", \"critical\": true, \"version\": \"1.94.2\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"1.94.2\\n  tailscale commit: 0a29cf18b56e478b9cd33af07755fcae90d5171a\\n  long version: 1.94.2-t0a29cf18b-g3f044c9f6\\n  other commit: 3f044c9f62dd0168c79f9ba7e67250b1dee06caa-dirty\\n  go version: go1.25.5\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool tailscale\", \"latest_version\": \"1.94.2\", \"update_ready\": false}, {\"id\": \"task\", \"name\": \"Taskwarrior\", \"category\": \"orchestration\", \"critical\": false, \"version\": \"3.48.0\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"3.48.0\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool task\", \"latest_version\": \"3.4.2\", \"update_ready\": false}, {\"id\": \"ruff\", \"name\": \"ruff\", \"category\": \"quality\", \"critical\": false, \"version\": \"0.15.5\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"venv_package=0.15.5; cmd=ruff 0.15.2\", \"update_script\": \"\", \"latest_version\": \"0.15.6\", \"update_ready\": true}, {\"id\": \"sqlite\", \"name\": \"SQLite Local Store\", \"category\": \"resilience\", \"critical\": false, \"version\": \"3.45.1\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"offline_queue_pending=0\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"node\", \"name\": \"Node.js\", \"category\": \"runtime\", \"critical\": true, \"version\": \"25.8.0\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"v25.8.0\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool node\", \"latest_version\": \"25.8.1\", \"update_ready\": true}, {\"id\": \"ollama\", \"name\": \"Ollama Runtime\", \"category\": \"runtime\", \"critical\": true, \"version\": \"0.17.7\", \"status\": \"ok\", \"health\": \"online\", \"details\": \"\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool ollama\", \"latest_version\": \"0.17.7\", \"update_ready\": false}, {\"id\": \"uv\", \"name\": \"uv\", \"category\": \"runtime\", \"critical\": false, \"version\": \"0.7.20\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"uv 0.7.20 (251420396 2025-07-09)\", \"update_script\": \"\", \"latest_version\": \"0.10.9\", \"update_ready\": true}, {\"id\": \"clawdbot_core\", \"name\": \"ClawdBOT Core Token\", \"category\": \"security\", \"critical\": true, \"version\": \"configured\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"ATLAS_CENTRAL_CORE / APPROVALS_CHAIN_SECRET\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"tmux\", \"name\": \"tmux\", \"category\": \"terminal\", \"critical\": false, \"version\": \"3.6\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"tmux 3.6a-win32\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool tmux\", \"latest_version\": \"3.6\", \"update_ready\": false}]}",
        "stderr_tail": "",
        "ms": 10177
      }
    },
    {
      "step": "refresh_software_watchdog",
      "ok": true,
      "ms": 41760,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "{\"ok\": true, \"generated_at\": \"2026-03-13T12:26:06.687450+00:00\", \"workspace\": \"C:\\\\ATLAS_PUSH\", \"scan_source\": \"atlas_software_watchdog.py\", \"catalog_version\": \"1.0\", \"summary\": {\"software_total\": 18, \"software_ok\": 8, \"software_warn\": 10, \"software_error\": 0, \"software_update_ready\": 7, \"candidates_total\": 2, \"candidates_available\": 2, \"drivers_total\": 4, \"drivers_ok\": 4, \"drivers_warn\": 0, \"drivers_error\": 0, \"dev_total\": 9, \"dev_ok\": 4, \"dev_warn\": 0, \"dev_error\": 5}, \"software\": [{\"id\": \"docker\", \"name\": \"Docker Desktop\", \"category\": \"containers\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"Docker.DockerDesktop\", \"installed\": true, \"version\": \"29.0.1\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Docker version 29.0.1, build eedd969\", \"network_probe\": \"https://docs.docker.com/desktop/\", \"network_available\": true, \"latency_ms\": 104, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"jq\", \"name\": \"jq\", \"category\": \"data\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"jqlang.jq\", \"installed\": true, \"version\": \"1.8.1\", \"latest_version\": \"1.8.1\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"jq-1.8.1\", \"network_probe\": \"https://jqlang.github.io/jq/\", \"network_available\": true, \"latency_ms\": 212, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"playwright_py\", \"name\": \"Playwright Python\", \"category\": \"dependency\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"playwright\", \"installed\": true, \"version\": \"1.46.0\", \"latest_version\": \"1.58.0\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"Version 1.46.0\", \"network_probe\": \"https://pypi.org/pypi/playwright/json\", \"network_available\": true, \"latency_ms\": 95, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"git\", \"name\": \"Git CLI\", \"category\": \"devops\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Git.Git\", \"installed\": true, \"version\": \"2.53.0.windows.1\", \"latest_version\": \"2.53.0.windows.2\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"git version 2.53.0.windows.1\", \"network_probe\": \"https://api.github.com/repos/git-for-windows/git/releases/latest\", \"network_available\": true, \"latency_ms\": 220, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"cloudflared\", \"name\": \"Cloudflare Tunnel\", \"category\": \"network\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Cloudflare.cloudflared\", \"installed\": true, \"version\": \"2025.11.1\", \"latest_version\": \"2026.3.0\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"cloudflared version 2025.11.1 (built 2025-11-07T16:30 UTC)\", \"network_probe\": \"https://api.github.com/repos/cloudflare/cloudflared/releases/latest\", \"network_available\": true, \"latency_ms\": 186, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"httpie\", \"name\": \"HTTPie\", \"category\": \"network\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"HTTPie.HTTPie\", \"installed\": true, \"version\": \"2025.2.0\", \"latest_version\": \"3.2.4\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"installed_no_path\", \"details\": \"- \\n                                                                                                                        \\n\\n   - \\n   \\\\ \\n   | \\n                                                                                                                        \\nNombre          Id            VersiÃ³n\\n--------------------------------------\\nHTTPie 2025.2.0 HTTPie.HTTPie 2025.2.0\", \"network_probe\": \"https://httpie.io\", \"network_available\": true, \"latency_ms\": 141, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"nmap\", \"name\": \"Nmap\", \"category\": \"network\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"Insecure.Nmap\", \"installed\": true, \"version\": \"5.3.5\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Nmap version 7.80 ( https://nmap.org )\\nPlatform: i686-pc-windows-windows\\nCompiled with: nmap-liblua-5.3.5 openssl-1.0.2s nmap-libssh2-1.8.2 nmap-libz-1.2.11 nmap-libpcre-7.6 Npcap-0.9982 nmap-libdnet-1.12 ipv6\\nCompiled without:\\nAvailable nsock engines: iocp poll select\", \"network_probe\": \"https://nmap.org\", \"network_available\": true, \"latency_ms\": 438, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"tailscale\", \"name\": \"Tailscale\", \"category\": \"network\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Tailscale.Tailscale\", \"installed\": true, \"version\": \"1.94.2\", \"latest_version\": \"1.94.2\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"1.94.2\\n  tailscale commit: 0a29cf18b56e478b9cd33af07755fcae90d5171a\\n  long version: 1.94.2-t0a29cf18b-g3f044c9f6\\n  other commit: 3f044c9f62dd0168c79f9ba7e67250b1dee06caa-dirty\\n  go version: go1.25.5\", \"network_probe\": \"https://api.github.com/repos/tailscale/tailscale/releases/latest\", \"network_available\": true, \"latency_ms\": 148, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"taskwarrior\", \"name\": \"Taskwarrior\", \"category\": \"orchestration\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"Task.Task\", \"installed\": true, \"version\": \"3.48.0\", \"latest_version\": \"3.4.2\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"3.48.0\", \"network_probe\": \"https://taskwarrior.org\", \"network_available\": true, \"latency_ms\": 92, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ruff\", \"name\": \"ruff\", \"category\": \"quality\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"ruff\", \"installed\": true, \"version\": \"0.15.2\", \"latest_version\": \"0.15.6\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"ruff 0.15.2\", \"network_probe\": \"https://pypi.org/pypi/ruff/json\", \"network_available\": true, \"latency_ms\": 90, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"node\", \"name\": \"Node.js\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"OpenJS.NodeJS\", \"installed\": true, \"version\": \"25.8.0\", \"latest_version\": \"25.8.1\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"v25.8.0\", \"network_probe\": \"https://nodejs.org/dist/index.json\", \"network_available\": true, \"latency_ms\": 126, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ollama\", \"name\": \"Ollama\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Ollama.Ollama\", \"installed\": true, \"version\": \"0.17.7\", \"latest_version\": \"0.17.7\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"ollama version is 0.17.7\", \"network_probe\": \"https://api.github.com/repos/ollama/ollama/releases/latest\", \"network_available\": true, \"latency_ms\": 165, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"python\", \"name\": \"Python Runtime\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Python.Python.3.12\", \"installed\": true, \"version\": \"3.11.9\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Python 3.11.9\", \"network_probe\": \"https://pypi.org/pypi/pip/json\", \"network_available\": true, \"latency_ms\": 131, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"pip\", \"name\": \"pip\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"pip\", \"install_target\": \"pip\", \"installed\": true, \"version\": \"25.3\", \"latest_version\": \"26.0.1\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"pip 25.3 from C:\\\\Users\\\\r6957\\\\AppData\\\\Local\\\\Programs\\\\Python\\\\Python311\\\\Lib\\\\site-packages\\\\pip (python 3.11)\", \"network_probe\": \"https://pypi.org/pypi/pip/json\", \"network_available\": true, \"latency_ms\": 104, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"uv\", \"name\": \"uv\", \"category\": \"runtime\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"astral-sh.uv\", \"installed\": true, \"version\": \"0.7.20\", \"latest_version\": \"0.10.9\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"uv 0.7.20 (251420396 2025-07-09)\", \"network_probe\": \"https://pypi.org/pypi/uv/json\", \"network_available\": true, \"latency_ms\": 93, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"tmux\", \"name\": \"tmux\", \"category\": \"terminal\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"arndawg.tmux-windows\", \"installed\": true, \"version\": \"3.6\", \"latest_version\": \"3.6\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"tmux 3.6a-win32\", \"network_probe\": \"https://github.com/tmux/tmux\", \"network_available\": true, \"latency_ms\": 449, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ccxt\", \"name\": \"CCXT\", \"category\": \"dependency\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"ccxt\", \"installed\": false, \"version\": \"\", \"latest_version\": \"4.5.43\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"not_detected\", \"details\": \"\", \"network_probe\": \"https://pypi.org/pypi/ccxt/json\", \"network_available\": true, \"latency_ms\": 123, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ytdlp\", \"name\": \"yt-dlp\", \"category\": \"media\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"yt-dlp\", \"installed\": false, \"version\": \"\", \"latest_version\": \"2026.3.13\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"not_detected\", \"details\": \"\", \"network_probe\": \"https://pypi.org/pypi/yt-dlp/json\", \"network_available\": true, \"latency_ms\": 95, \"network_status\": \"network_ok\", \"network_error\": null}], \"network_candidates\": [{\"id\": \"ccxt\", \"name\": \"CCXT\", \"category\": \"dependency\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"ccxt\", \"network_available\": true, \"latency_ms\": 123, \"network_error\": null, \"latest_version\": \"4.5.43\", \"score\": 90, \"status\": \"ready\"}, {\"id\": \"ytdlp\", \"name\": \"yt-dlp\", \"category\": \"media\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"yt-dlp\", \"network_available\": true, \"latency_ms\": 95, \"network_error\": null, \"latest_version\": \"2026.3.13\", \"score\": 90, \"status\": \"ready\"}], \"drivers\": [{\"id\": \"audio_driver\", \"name\": \"Audio Driver\", \"critical\": false, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"10.0.26100.1\", \"matches\": [{\"name\": \"Microsoft Streaming Quality Manager Proxy\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"MEDIA\"}, {\"name\": \"Microsoft Streaming Service Proxy\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"MEDIA\"}, {\"name\": \"NVIDIA Virtual Audio Device (Wave Extensible) (WDM)\", \"version\": \"4.65.0.3\", \"provider\": \"NVIDIA\", \"class\": \"MEDIA\"}, {\"name\": \"Realtek High Definition Audio\", \"version\": \"6.0.9549.1\", \"provider\": \"Realtek Semiconductor Corp.\", \"class\": \"MEDIA\"}, {\"name\": \"Microsoft Bluetooth Hands-Free Audio device\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"MEDIA\"}], \"count\": 11}, {\"id\": \"bluetooth_driver\", \"name\": \"Bluetooth Driver\", \"critical\": false, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"10.0.26100.7920\", \"matches\": [{\"name\": \"Bluetooth LE Generic Attribute Service\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Device Information Service\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Generic Attribute Profile\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Generic Access Profile\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Bluetooth LE Device\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}], \"count\": 20}, {\"id\": \"gpu_display_driver\", \"name\": \"GPU Display Driver\", \"critical\": true, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"32.0.15.9155\", \"matches\": [{\"name\": \"NVIDIA GeForce GTX 1660 SUPER\", \"version\": \"32.0.15.9155\", \"date\": \"/Date(1765238400000)/\"}], \"count\": 1}, {\"id\": \"network_adapter_driver\", \"name\": \"Network Adapter Driver\", \"critical\": true, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"10.0.26100.1\", \"matches\": [{\"name\": \"Hyper-V Virtual Switch Extension Adapter\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"WAN Miniport (Network Monitor)\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"WAN Miniport (IPv6)\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"WAN Miniport (IP)\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"WAN Miniport (PPPOE)\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}], \"count\": 18}], \"development_stack\": [{\"id\": \"ans_scheduler\", \"name\": \"ANS Scheduler Job\", \"type\": \"scheduler_job\", \"critical\": true, \"status\": \"ok\", \"health\": \"success\", \"version\": \"\", \"details\": \"job=ans_cycle\"}, {\"id\": \"nexus_api\", \"name\": \"ATLAS NEXUS API\", \"type\": \"service\", \"critical\": false, \"status\": \"ok\", \"health\": \"online\", \"version\": \"2.0.0\", \"details\": \"\"}, {\"id\": \"robot_api\", \"name\": \"ATLAS Robot API\", \"type\": \"service\", \"critical\": false, \"status\": \"ok\", \"health\": \"online\", \"version\": \"\", \"details\": \"\"}, {\"id\": \"triada_last_cycle\", \"name\": \"Triada Last Cycle\", \"type\": \"log_file\", \"critical\": true, \"status\": \"ok\", \"health\": \"present\", \"version\": \"2026-03-13T12:25:13.296731+00:00\", \"details\": \"Asimilación Exitosa\"}, {\"id\": \"push_api\", \"name\": \"ATLAS PUSH API\", \"type\": \"service\", \"critical\": true, \"status\": \"error\", \"health\": \"offline\", \"version\": \"\", \"details\": \"timed out\"}, {\"id\": \"makeplay_last_scan\", \"name\": \"MakePlay Last Scan\", \"type\": \"makeplay_last_scan\", \"critical\": true, \"status\": \"error\", \"health\": \"empty\", \"version\": \"\", \"details\": \"no_makeplay_scan_data\"}, {\"id\": \"makeplay_scheduler\", \"name\": \"MakePlay Scheduler Job\", \"type\": \"scheduler_job\", \"critical\": true, \"status\": \"error\", \"health\": \"failed\", \"version\": \"\", \"details\": \"job=makeplay_scanner\"}, {\"id\": \"repo_monitor_scheduler\", \"name\": \"Repo Monitor Scheduler Job\", \"type\": \"scheduler_job\", \"critical\": true, \"status\": \"error\", \"health\": \"paused\", \"version\": \"\", \"details\": \"job=repo_monitor_cycle\"}, {\"id\": \"repo_update_status\", \"name\": \"Repo Update Status\", \"type\": \"repo_update\", \"critical\": true, \"status\": \"error\", \"health\": \"unknown\", \"version\": \"\", \"details\": \"head= remote=\"}]}",
        "stderr_tail": "",
        "ms": 41760
      }
    }
  ]
}
```