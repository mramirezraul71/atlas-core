# ATLAS Software Maintenance Report

- Start: `2026-03-05T05:11:12.035619+00:00`
- End: `2026-03-05T05:12:40.146814+00:00`
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
  - Detail: `{"ok": true, "generated_at": "2026-03-05T05:12:15.178599+00:00", "workspace": "C:\\ATLAS_PUSH", "scan_source": "atlas_tools_watchdog.py", "summary": {"total": 18, "ok": 14, "warn": 3, "error": 1, "upg`
- `refresh_software_watchdog`: **OK**
  - Detail: `{"ok": true, "generated_at": "2026-03-05T05:12:39.974678+00:00", "workspace": "C:\\ATLAS_PUSH", "scan_source": "atlas_software_watchdog.py", "catalog_version": "1.0", "summary": {"software_total": 18,`

## Raw JSON
```json
{
  "ok": true,
  "started_at": "2026-03-05T05:11:12.035619+00:00",
  "finished_at": "2026-03-05T05:12:40.146814+00:00",
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
      "ms": 189,
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
      "ms": 1201,
      "details": {
        "ok": true,
        "snapshot": {
          "health_score": 85,
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
            "audit_writable": false,
            "scheduler_alive": true,
            "llm_reachable": true,
            "avg_latency_ms": 0.0,
            "error_rate": 0.0,
            "ans_open_incidents": 0,
            "nervous_score": 85,
            "nervous_points_total": 0,
            "nervous_open_signals": 0,
            "nervous_by_severity_points": {
              "low": 0,
              "med": 0,
              "high": 0,
              "critical": 0
            },
            "active_port": 8791,
            "version": "1.0.0-alpha-612-g696ee9806-dirty",
            "channel": "canary"
          },
          "webhook_pushed": false,
          "ts": "2026-03-05T05:11:13.426790+00:00"
        }
      }
    },
    {
      "step": "ans_cycle",
      "ok": true,
      "ms": 28258,
      "details": {
        "ok": true,
        "issues_count": 4,
        "actions_taken": 2,
        "report_path": "C:\\ATLAS_PUSH\\snapshots\\ans\\ANS_REPORT_20260305_051141.md"
      }
    },
    {
      "step": "triada_run_once",
      "ok": true,
      "ms": 22669,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "",
        "stderr_tail": "2026-03-05 00:11:41 [atlas.evolution] INFO Ejecutando un solo ciclo de la Tríada (PyPI | GitHub | HF)...\n2026-03-05 00:11:59 [atlas.evolution] INFO Tríada de Crecimiento: PyPI | GitHub | Hugging Face\n2026-03-05 00:12:02 [atlas.evolution] INFO [HF] robbyant/lingbot-vla-4b-depth (query=VLA) hash=e41a5ac6652d6655\n2026-03-05 00:12:02 [atlas.evolution] INFO [GitHub] mramirezraul71/atlas-core tag= commits=3 tools=[]\n2026-03-05 00:12:02 [atlas.evolution] INFO [HF] robbyant/lingbot-vla-4b (query=VLA) hash=e277aeaa4310ab69\n2026-03-05 00:12:02 [atlas.evolution] INFO [HF] mbreuss/flower_vla_pret (query=VLA) hash=ec1ea3b4d007033e\n2026-03-05 00:12:04 [atlas.evolution] INFO Ciclo completado.",
        "ms": 22669
      }
    },
    {
      "step": "repo_monitor_cycle",
      "ok": true,
      "ms": 759,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "",
        "stderr_tail": "",
        "ms": 759
      }
    },
    {
      "step": "repo_update_cycle",
      "ok": true,
      "ms": 973,
      "details": {
        "ok": true,
        "phase": "check",
        "data": {
          "ok": true,
          "enabled": true,
          "branch": "dev",
          "head_commit": "696ee980675ab90f3646e63967089267b267a9d8",
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
          "ms": 547,
          "error": null,
          "snapshot": {
            "status_output": "m _external/RAULI-VISION\n m _external/rauli-panaderia\n M atlas_adapter/atlas_http_api.py\n M atlas_adapter/static/v4/app.js\n M atlas_adapter/static/v4/components/landing.js\n M atlas_adapter/static/v4/components/megamenu.js\n M atlas_adapter/static/v4/modules/tools_menu.js\n M atlas_master_registry.json\n M tools/atlas_agent/memory/episodes.sqlite\n?? atlas_adapter/static/v4/modules/software_center.js\n?? reports/software_maintenance_20260305-001012.md\n?? scripts/atlas_software_apply_all_background.ps1\n?? scripts/atlas_software_apply_item_background.ps1\n?? scripts/atlas_software_maintenance_cycle.py\n?? scripts/atlas_software_watchdog.py\n?? software_hub/",
            "diff_output": "diff --git a/_external/RAULI-VISION b/_external/RAULI-VISION\n--- a/_external/RAULI-VISION\n+++ b/_external/RAULI-VISION\n@@ -1 +1 @@\n-Subproject commit 3225bfdf0be93b30dcbafa20b145b71a5cdb2e7a\n+Subproject commit 3225bfdf0be93b30dcbafa20b145b71a5cdb2e7a-dirty\ndiff --git a/_external/rauli-panaderia b/_external/rauli-panaderia\n--- a/_external/rauli-panaderia\n+++ b/_external/rauli-panaderia\n@@ -1 +1 @@\n-Subproject commit 6a99384b3a21f0657372c0fff789fbdd54474ed8\n+Subproject commit 6a99384b3a21f0657372c0fff789fbdd54474ed8-dirty\ndiff --git a/atlas_adapter/atlas_http_api.py b/atlas_adapter/atlas_http_api.py\nindex 0504db2c6..e188019ef 100644\n--- a/atlas_adapter/atlas_http_api.py\n+++ b/atlas_adapter/atlas_http_api.py\n@@ -9760,12 +9760,14 @@ _tools_discovery_refresh_running = False\n def _read_json_dict(path: Path) -> Optional[dict]:\n     if not path.exists():\n         return None\n-    try:\n-        data = json.loads(path.read_text(encoding=\"utf-8-sig\", errors=\"replace\"))\n-        if isinstance(data, dict):\n-            return data\n-    except Exception:\n-        return None\n+    for _ in range(3):\n+        try:\n+            data = json.loads(path.read_text(encoding=\"utf-8-sig\", errors=\"replace\"))\n+            if isinstance(data, dict):\n+                return data\n+        except Exception:\n+            # Tolerate transient partial writes and retry quickly.\n+            time.sleep(0.08)\n     return None\n \n \n@@ -11504,6 +11506,590 @@ def tools_job_retry(job_id: str):\n         return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))\n \n \n+# --- Software Hub ---\n+class SoftwareWatchdogBody(BaseModel):\n+    force: Optional[bool] = False\n+\n+\n+class SoftwareApplyAllBody(BaseModel):\n+    background: Optional[bool] = True\n+    include_optional: Optional[bool] = True\n+\n+\n+class SoftwareMaintenanceBody(BaseModel):\n+    background: Optional[bool] = True\n+    apply_repo: Optional[bool] = False\n+\n+\n+_SOFTWARE_HUB_DIR = (BASE_DIR / \"software_hub\").resolve()\n+_SOFTWARE_REGISTRY_FILE = (_SOFTWARE_HUB_DIR / \"software_registry.json\").resolve()\n+_SOFTWARE_MAINT_LATEST_FILE = (BASE_DIR / \"logs\" / \"software_maintenance\" / \"latest_cycle.json\").resolve()\n+_SOFTWARE_MENU_CACHE_TTL_SEC = _env_int(\"ATLAS_SOFTWARE_MENU_CACHE_TTL_SEC\", 30, 8)\n+_SOFTWARE_MENU_STALE_SEC = _env_int(\n+    \"ATLAS_SOFTWARE_MENU_STALE_SEC\", 300, _SOFTWARE_MENU_CACHE_TTL_SEC\n+)\n+_software_watchdog_refresh_lock = threading.Lock()\n+_software_watchdog_refresh_running = False\n+\n+\n+def _run_software_watchdog_scan(force: bool = False, timeout: int = 180) -> tuple[bool, dict]:\n+    script = (BASE_DIR / \"scripts\" / \"atlas_software_watchdog.py\").resolve()\n+    if not script.exists():\n+        return False, {\"error\": f\"software watchdog script missing: {script}\"}\n+    cmd = [sys.executable, str(script)]\n+    if force:\n+        cmd.append(\"--force\")\n+    ok, payload, _tail = _run_local_json_cmd(cmd, timeout=timeout)\n+    if not isinstance(payload, dict):\n+        payload = {\"ok\": bool(ok), \"error\": \"software_watchdog_invalid_payload\"}\n+    return bool(ok), payload\n+\n+\n+def _spawn_software_watchdog_refresh(force: bool = False) -> bool:\n+    global _software_watchdog_refresh_running\n+    with _software_watchdog_refresh_lock:\n+        if _software_watchdog_refresh_running:\n+            return False\n+        _software_watchdog_refresh_running = True\n+\n+    def _job() -> None:\n+        global _software_watchdog_refresh_running\n+        try:\n+            _run_software_watchdog_scan(force=force, timeout=240)\n+        except Exception:\n+            pass\n+        finally:\n+            with _software_watchdog_refresh_lock:\n+                _software_watchdog_refresh_running = False\n+\n+    threading.Thread(target=_job, name=\"atlas-software-watchdog-refresh\", daemon=True).start()\n+    return True\n+\n+\n+@app.get(\"/api/software/menu\", tags=[\"Software\"])\n+def software_menu_inventory(refresh: bool = False):\n+    \"\"\"Inventario extendido de software (instalado + red + drivers + stack dev).\"\"\"\n+    t0 = time.perf_counter()\n+    now_ts = time.time()\n+    payload = _read_json_dict(_SOFTWARE_REGISTR"
          }
        }
      }
    },
    {
      "step": "refresh_tools_watchdog",
      "ok": true,
      "ms": 9109,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "{\"ok\": true, \"generated_at\": \"2026-03-05T05:12:15.178599+00:00\", \"workspace\": \"C:\\\\ATLAS_PUSH\", \"scan_source\": \"atlas_tools_watchdog.py\", \"summary\": {\"total\": 18, \"ok\": 14, \"warn\": 3, \"error\": 1, \"upgrade_ready\": 2}, \"offline_queue\": {\"pending\": 0, \"db_path\": \"C:\\\\ATLAS_PUSH\\\\logs\\\\atlas_comms_hub.sqlite\", \"resync_script\": \"scripts\\\\atlas_resync_flow.ps1\"}, \"channels\": {\"panaderia\": \"http://127.0.0.1:3001\", \"vision\": \"http://127.0.0.1:3000\", \"push\": \"http://127.0.0.1:8791\"}, \"git\": {\"branch\": \"dev\", \"is_protected\": false, \"protected_branches\": [\"main\", \"master\", \"prod\", \"production\", \"release\"], \"allow_protected_updates\": false}, \"security\": {\"core_token_env\": false, \"approval_secret_env\": true, \"encryption_mode\": \"APPROVALS_CHAIN_SECRET\"}, \"tools\": [{\"id\": \"jq\", \"name\": \"jq\", \"category\": \"data\", \"critical\": false, \"version\": \"1.8.1\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"jq-1.8.1\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"ccxt\", \"name\": \"CCXT (Trading)\", \"category\": \"dependency\", \"critical\": false, \"version\": \"4.5.40\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool ccxt\", \"latest_version\": \"4.5.40\", \"update_ready\": false}, {\"id\": \"playwright_py\", \"name\": \"Playwright Python\", \"category\": \"dependency\", \"critical\": false, \"version\": \"1.58.0\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool playwright_py\", \"latest_version\": \"1.58.0\", \"update_ready\": false}, {\"id\": \"puppeteer\", \"name\": \"Puppeteer Node\", \"category\": \"dependency\", \"critical\": false, \"version\": \"24.38.0\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"tools/atlas_actuators\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool puppeteer\", \"latest_version\": \"24.38.0\", \"update_ready\": false}, {\"id\": \"git\", \"name\": \"Git CLI\", \"category\": \"devops\", \"critical\": true, \"version\": \"2.53.0.windows.1\", \"status\": \"ok\", \"health\": \"online\", \"details\": \"git version 2.53.0.windows.1\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool git\", \"latest_version\": \"2.53.0.windows.1\", \"update_ready\": false}, {\"id\": \"yt-dlp\", \"name\": \"yt-dlp\", \"category\": \"media\", \"critical\": false, \"version\": \"2026.3.3\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"venv_package=2026.3.3\", \"update_script\": \"\", \"latest_version\": \"2026.3.3\", \"update_ready\": false}, {\"id\": \"cloudflared\", \"name\": \"Cloudflare Tunnel\", \"category\": \"network\", \"critical\": true, \"version\": \"2025.11.1\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"cloudflared version 2025.11.1 (built 2025-11-07T16:30 UTC)\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool cloudflared\", \"latest_version\": \"2026.2.0\", \"update_ready\": true}, {\"id\": \"httpie\", \"name\": \"HTTPie\", \"category\": \"network\", \"critical\": false, \"version\": \"2025.2.0\", \"status\": \"warn\", \"health\": \"installed_no_path\", \"details\": \"installed via winget (PATH pendiente)\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"nmap\", \"name\": \"Nmap\", \"category\": \"network\", \"critical\": false, \"version\": \"5.3.5\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Nmap version 7.80 ( https://nmap.org )\\nPlatform: i686-pc-windows-windows\\nCompiled with: nmap-liblua-5.3.5 openssl-1.0.2s nmap-libssh2-1.8.2 nmap-libz-1.2.11 nmap-libpcre-7.6 Npcap-0.9982 nmap-libdnet-1.12 ipv6\\nCompiled without:\\nAvailable nsock engines: iocp poll select\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"tailscale\", \"name\": \"Tailscale Backup Tunnel\", \"category\": \"network\", \"critical\": true, \"version\": \"\", \"status\": \"error\", \"health\": \"not_detected\", \"details\": \"[WinError 2] El sistema no puede encontrar el archivo especificado\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool tailscale\", \"latest_version\": \"1.94.2\", \"update_ready\": false}, {\"id\": \"task\", \"name\": \"Taskwarrior\", \"category\": \"orchestration\", \"critical\": false, \"version\": \"3.48.0\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"3.48.0\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"ruff\", \"name\": \"ruff\", \"category\": \"quality\", \"critical\": false, \"version\": \"0.15.4\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"venv_package=0.15.4; cmd=ruff 0.15.2\", \"update_script\": \"\", \"latest_version\": \"0.15.4\", \"update_ready\": false}, {\"id\": \"sqlite\", \"name\": \"SQLite Local Store\", \"category\": \"resilience\", \"critical\": false, \"version\": \"3.45.1\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"offline_queue_pending=0\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"node\", \"name\": \"Node.js\", \"category\": \"runtime\", \"critical\": true, \"version\": \"25.8.0\", \"status\": \"ok\", \"health\": \"online\", \"details\": \"v25.8.0\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool node\", \"latest_version\": \"25.8.0\", \"update_ready\": false}, {\"id\": \"ollama\", \"name\": \"Ollama Runtime\", \"category\": \"runtime\", \"critical\": true, \"version\": \"0.17.6\", \"status\": \"ok\", \"health\": \"online\", \"details\": \"\", \"update_script\": \"scripts\\\\atlas_tool_update.ps1 -Tool ollama\", \"latest_version\": \"0.17.6\", \"update_ready\": false}, {\"id\": \"uv\", \"name\": \"uv\", \"category\": \"runtime\", \"critical\": false, \"version\": \"0.7.20\", \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"uv 0.7.20 (251420396 2025-07-09)\", \"update_script\": \"\", \"latest_version\": \"0.10.8\", \"update_ready\": true}, {\"id\": \"clawdbot_core\", \"name\": \"ClawdBOT Core Token\", \"category\": \"security\", \"critical\": true, \"version\": \"configured\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"ATLAS_CENTRAL_CORE / APPROVALS_CHAIN_SECRET\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}, {\"id\": \"tmux\", \"name\": \"tmux\", \"category\": \"terminal\", \"critical\": false, \"version\": \"3.6\", \"status\": \"ok\", \"health\": \"ready\", \"details\": \"tmux 3.6a-win32\", \"update_script\": \"\", \"latest_version\": \"\", \"update_ready\": false}]}",
        "stderr_tail": "",
        "ms": 9109
      }
    },
    {
      "step": "refresh_software_watchdog",
      "ok": true,
      "ms": 24948,
      "details": {
        "ok": true,
        "returncode": 0,
        "stdout_tail": "{\"ok\": true, \"generated_at\": \"2026-03-05T05:12:39.974678+00:00\", \"workspace\": \"C:\\\\ATLAS_PUSH\", \"scan_source\": \"atlas_software_watchdog.py\", \"catalog_version\": \"1.0\", \"summary\": {\"software_total\": 18, \"software_ok\": 9, \"software_warn\": 9, \"software_error\": 0, \"software_update_ready\": 5, \"candidates_total\": 2, \"candidates_available\": 2, \"drivers_total\": 4, \"drivers_ok\": 4, \"drivers_warn\": 0, \"drivers_error\": 0, \"dev_total\": 9, \"dev_ok\": 4, \"dev_warn\": 2, \"dev_error\": 3}, \"software\": [{\"id\": \"docker\", \"name\": \"Docker Desktop\", \"category\": \"containers\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"Docker.DockerDesktop\", \"installed\": true, \"version\": \"29.0.1\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Docker version 29.0.1, build eedd969\", \"network_probe\": \"https://docs.docker.com/desktop/\", \"network_available\": true, \"latency_ms\": 227, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"jq\", \"name\": \"jq\", \"category\": \"data\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"jqlang.jq\", \"installed\": true, \"version\": \"1.8.1\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"jq-1.8.1\", \"network_probe\": \"https://jqlang.github.io/jq/\", \"network_available\": true, \"latency_ms\": 211, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"playwright_py\", \"name\": \"Playwright Python\", \"category\": \"dependency\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"playwright\", \"installed\": true, \"version\": \"1.46.0\", \"latest_version\": \"1.58.0\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"Version 1.46.0\", \"network_probe\": \"https://pypi.org/pypi/playwright/json\", \"network_available\": true, \"latency_ms\": 94, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"git\", \"name\": \"Git CLI\", \"category\": \"devops\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Git.Git\", \"installed\": true, \"version\": \"2.53.0.windows.1\", \"latest_version\": \"2.53.0.windows.1\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"git version 2.53.0.windows.1\", \"network_probe\": \"https://api.github.com/repos/git-for-windows/git/releases/latest\", \"network_available\": true, \"latency_ms\": 169, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"cloudflared\", \"name\": \"Cloudflare Tunnel\", \"category\": \"network\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Cloudflare.cloudflared\", \"installed\": true, \"version\": \"2025.11.1\", \"latest_version\": \"2026.2.0\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"cloudflared version 2025.11.1 (built 2025-11-07T16:30 UTC)\", \"network_probe\": \"https://api.github.com/repos/cloudflare/cloudflared/releases/latest\", \"network_available\": true, \"latency_ms\": 181, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"httpie\", \"name\": \"HTTPie\", \"category\": \"network\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"HTTPie.HTTPie\", \"installed\": true, \"version\": \"2025.2.0\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"installed_no_path\", \"details\": \"- \\n                                                                                                                        \\n\\n   - \\n   \\\\ \\n                                                                                                                        \\nNombre          Id            VersiÃ³n\\n--------------------------------------\\nHTTPie 2025.2.0 HTTPie.HTTPie 2025.2.0\", \"network_probe\": \"https://httpie.io\", \"network_available\": true, \"latency_ms\": 196, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"nmap\", \"name\": \"Nmap\", \"category\": \"network\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"Insecure.Nmap\", \"installed\": true, \"version\": \"5.3.5\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Nmap version 7.80 ( https://nmap.org )\\nPlatform: i686-pc-windows-windows\\nCompiled with: nmap-liblua-5.3.5 openssl-1.0.2s nmap-libssh2-1.8.2 nmap-libz-1.2.11 nmap-libpcre-7.6 Npcap-0.9982 nmap-libdnet-1.12 ipv6\\nCompiled without:\\nAvailable nsock engines: iocp poll select\", \"network_probe\": \"https://nmap.org\", \"network_available\": true, \"latency_ms\": 377, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"tailscale\", \"name\": \"Tailscale\", \"category\": \"network\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Tailscale.Tailscale\", \"installed\": true, \"version\": \"1.94.2\", \"latest_version\": \"1.94.2\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"installed_no_path\", \"details\": \"- \\n                                                                                                                        \\n\\n   - \\n   \\\\ \\n                                                                                                                        \\nNombre    Id                  VersiÃ³n\\n-------------------------------------\\nTailscale Tailscale.Tailscale 1.94.2\", \"network_probe\": \"https://api.github.com/repos/tailscale/tailscale/releases/latest\", \"network_available\": true, \"latency_ms\": 175, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"taskwarrior\", \"name\": \"Taskwarrior\", \"category\": \"orchestration\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"Task.Task\", \"installed\": true, \"version\": \"3.48.0\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"3.48.0\", \"network_probe\": \"https://taskwarrior.org\", \"network_available\": true, \"latency_ms\": 121, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ruff\", \"name\": \"ruff\", \"category\": \"quality\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"ruff\", \"installed\": true, \"version\": \"0.15.2\", \"latest_version\": \"0.15.4\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"ruff 0.15.2\", \"network_probe\": \"https://pypi.org/pypi/ruff/json\", \"network_available\": true, \"latency_ms\": 96, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"node\", \"name\": \"Node.js\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"OpenJS.NodeJS\", \"installed\": true, \"version\": \"25.8.0\", \"latest_version\": \"25.8.0\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"v25.8.0\", \"network_probe\": \"https://nodejs.org/dist/index.json\", \"network_available\": true, \"latency_ms\": 138, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ollama\", \"name\": \"Ollama\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Ollama.Ollama\", \"installed\": true, \"version\": \"0.17.6\", \"latest_version\": \"0.17.6\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"ollama version is 0.17.6\", \"network_probe\": \"https://api.github.com/repos/ollama/ollama/releases/latest\", \"network_available\": true, \"latency_ms\": 201, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"python\", \"name\": \"Python Runtime\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"winget\", \"install_target\": \"Python.Python.3.12\", \"installed\": true, \"version\": \"3.11.9\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"Python 3.11.9\", \"network_probe\": \"https://pypi.org/pypi/pip/json\", \"network_available\": true, \"latency_ms\": 120, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"pip\", \"name\": \"pip\", \"category\": \"runtime\", \"critical\": true, \"install_method\": \"pip\", \"install_target\": \"pip\", \"installed\": true, \"version\": \"25.3\", \"latest_version\": \"26.0.1\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"pip 25.3 from C:\\\\Users\\\\r6957\\\\AppData\\\\Local\\\\Programs\\\\Python\\\\Python311\\\\Lib\\\\site-packages\\\\pip (python 3.11)\", \"network_probe\": \"https://pypi.org/pypi/pip/json\", \"network_available\": true, \"latency_ms\": 100, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"uv\", \"name\": \"uv\", \"category\": \"runtime\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"astral-sh.uv\", \"installed\": true, \"version\": \"0.7.20\", \"latest_version\": \"0.10.8\", \"update_ready\": true, \"status\": \"warn\", \"health\": \"upgrade_ready\", \"details\": \"uv 0.7.20 (251420396 2025-07-09)\", \"network_probe\": \"https://pypi.org/pypi/uv/json\", \"network_available\": true, \"latency_ms\": 118, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"tmux\", \"name\": \"tmux\", \"category\": \"terminal\", \"critical\": false, \"install_method\": \"winget\", \"install_target\": \"arndawg.tmux-windows\", \"installed\": true, \"version\": \"3.6\", \"latest_version\": \"\", \"update_ready\": false, \"status\": \"ok\", \"health\": \"ready\", \"details\": \"tmux 3.6a-win32\", \"network_probe\": \"https://github.com/tmux/tmux\", \"network_available\": true, \"latency_ms\": 429, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ccxt\", \"name\": \"CCXT\", \"category\": \"dependency\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"ccxt\", \"installed\": false, \"version\": \"\", \"latest_version\": \"4.5.40\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"not_detected\", \"details\": \"\", \"network_probe\": \"https://pypi.org/pypi/ccxt/json\", \"network_available\": true, \"latency_ms\": 111, \"network_status\": \"network_ok\", \"network_error\": null}, {\"id\": \"ytdlp\", \"name\": \"yt-dlp\", \"category\": \"media\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"yt-dlp\", \"installed\": false, \"version\": \"\", \"latest_version\": \"2026.3.3\", \"update_ready\": false, \"status\": \"warn\", \"health\": \"not_detected\", \"details\": \"\", \"network_probe\": \"https://pypi.org/pypi/yt-dlp/json\", \"network_available\": true, \"latency_ms\": 107, \"network_status\": \"network_ok\", \"network_error\": null}], \"network_candidates\": [{\"id\": \"ccxt\", \"name\": \"CCXT\", \"category\": \"dependency\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"ccxt\", \"network_available\": true, \"latency_ms\": 111, \"network_error\": null, \"latest_version\": \"4.5.40\", \"score\": 90, \"status\": \"ready\"}, {\"id\": \"ytdlp\", \"name\": \"yt-dlp\", \"category\": \"media\", \"critical\": false, \"install_method\": \"pip\", \"install_target\": \"yt-dlp\", \"network_available\": true, \"latency_ms\": 107, \"network_error\": null, \"latest_version\": \"2026.3.3\", \"score\": 90, \"status\": \"ready\"}], \"drivers\": [{\"id\": \"audio_driver\", \"name\": \"Audio Driver\", \"critical\": false, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"10.0.26100.1\", \"matches\": [{\"name\": \"Microsoft Streaming Service Proxy\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"MEDIA\"}, {\"name\": \"NVIDIA Virtual Audio Device (Wave Extensible) (WDM)\", \"version\": \"4.65.0.3\", \"provider\": \"NVIDIA\", \"class\": \"MEDIA\"}, {\"name\": \"Realtek High Definition Audio\", \"version\": \"6.0.9549.1\", \"provider\": \"Realtek Semiconductor Corp.\", \"class\": \"MEDIA\"}, {\"name\": \"Microsoft Bluetooth Hands-Free Audio device\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"MEDIA\"}, {\"name\": \"Microsoft Bluetooth Hands-Free Audio device\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"MEDIA\"}], \"count\": 10}, {\"id\": \"bluetooth_driver\", \"name\": \"Bluetooth Driver\", \"critical\": false, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"10.0.26100.7920\", \"matches\": [{\"name\": \"Bluetooth LE Generic Attribute Service\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Device Information Service\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Generic Attribute Profile\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Generic Access Profile\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}, {\"name\": \"Bluetooth LE Device\", \"version\": \"10.0.26100.7920\", \"provider\": \"Microsoft\", \"class\": \"BLUETOOTH\"}], \"count\": 20}, {\"id\": \"gpu_display_driver\", \"name\": \"GPU Display Driver\", \"critical\": true, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"32.0.15.9155\", \"matches\": [{\"name\": \"NVIDIA GeForce GTX 1660 SUPER\", \"version\": \"32.0.15.9155\", \"date\": \"/Date(1765238400000)/\"}], \"count\": 1}, {\"id\": \"network_adapter_driver\", \"name\": \"Network Adapter Driver\", \"critical\": true, \"status\": \"ok\", \"health\": \"ready\", \"detected\": true, \"version\": \"0.14.0.0\", \"matches\": [{\"name\": \"Tailscale Tunnel\", \"version\": \"0.14.0.0\", \"provider\": \"WireGuard LLC\", \"class\": \"NET\"}, {\"name\": \"Microsoft KM-TEST Loopback Adapter\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"Hyper-V Virtual Switch Extension Adapter\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"WAN Miniport (Network Monitor)\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}, {\"name\": \"WAN Miniport (IPv6)\", \"version\": \"10.0.26100.1\", \"provider\": \"Microsoft\", \"class\": \"NET\"}], \"count\": 18}], \"development_stack\": [{\"id\": \"ans_scheduler\", \"name\": \"ANS Scheduler Job\", \"type\": \"scheduler_job\", \"critical\": true, \"status\": \"ok\", \"health\": \"success\", \"version\": \"\", \"details\": \"job=ans_cycle\"}, {\"id\": \"nexus_api\", \"name\": \"ATLAS NEXUS API\", \"type\": \"service\", \"critical\": false, \"status\": \"ok\", \"health\": \"online\", \"version\": \"2.0.0\", \"details\": \"\"}, {\"id\": \"repo_monitor_scheduler\", \"name\": \"Repo Monitor Scheduler Job\", \"type\": \"scheduler_job\", \"critical\": true, \"status\": \"ok\", \"health\": \"queued\", \"version\": \"\", \"details\": \"job=repo_monitor_cycle\"}, {\"id\": \"triada_last_cycle\", \"name\": \"Triada Last Cycle\", \"type\": \"log_file\", \"critical\": true, \"status\": \"ok\", \"health\": \"present\", \"version\": \"2026-03-05T05:12:04.282823+00:00\", \"details\": \"Asimilación Exitosa\"}, {\"id\": \"push_api\", \"name\": \"ATLAS PUSH API\", \"type\": \"service\", \"critical\": true, \"status\": \"error\", \"health\": \"offline\", \"version\": \"\", \"details\": \"timed out\"}, {\"id\": \"robot_api\", \"name\": \"ATLAS Robot API\", \"type\": \"service\", \"critical\": false, \"status\": \"warn\", \"health\": \"offline\", \"version\": \"\", \"details\": \"<urlopen error [WinError 10061] No se puede establecer una conexión ya que el equipo de destino denegó expresamente dicha conexión>\"}, {\"id\": \"makeplay_last_scan\", \"name\": \"MakePlay Last Scan\", \"type\": \"makeplay_last_scan\", \"critical\": true, \"status\": \"error\", \"health\": \"empty\", \"version\": \"\", \"details\": \"no_makeplay_scan_data\"}, {\"id\": \"makeplay_scheduler\", \"name\": \"MakePlay Scheduler Job\", \"type\": \"scheduler_job\", \"critical\": true, \"status\": \"error\", \"health\": \"failed\", \"version\": \"\", \"details\": \"job=makeplay_scanner\"}, {\"id\": \"repo_update_status\", \"name\": \"Repo Update Status\", \"type\": \"repo_update\", \"critical\": true, \"status\": \"warn\", \"health\": \"update_ready\", \"version\": \"dev\", \"details\": \"head=696ee980 remote=eb6525bd\"}]}",
        "stderr_tail": "",
        "ms": 24948
      }
    }
  ]
}
```