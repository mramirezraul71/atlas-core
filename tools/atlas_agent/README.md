# ATLAS Agent (`tools/atlas_agent`)

Autonomous local agent for ATLAS, inspired by Clawdbot-style loops:
- think with OpenAI
- choose a tool action
- execute locally
- observe result
- continue until final answer

Current implementation includes:
- multi-objective planning + dynamic reprioritization
- safe/aggressive approval policy for mutating tools
- episodic semantic memory (SQLite + embeddings)
- deterministic run artifacts with tool diffs/hashes
- HTTP integration in PUSH API (`/api/agent/autonomous/*`)

## Requirements

1. `OPENAI_API_KEY` available in environment.
2. Python env with `openai` package:

```powershell
.\.venv\Scripts\python.exe -m pip install openai
```

## Quick Start

```powershell
.\.venv\Scripts\python.exe tools/atlas_agent/main.py `
  --goal "Audit dashboard v4 polling and report root cause if stale" `
  --max-steps 10 `
  --model gpt-4o-mini
```

Aggressive mode with explicit approvals:

```powershell
.\.venv\Scripts\python.exe tools/atlas_agent/main.py `
  --goal "Implement endpoint and validate smoke tests" `
  --mode aggressive `
  --approved-tool write_file `
  --approval-override run_shell=true
```

## Safety and Scope

- Tools are constrained to the configured workspace (`--workspace` or repo root by default).
- `run_shell` supports guarded execution (basic deny list for destructive commands).
- `safe` mode: `write_file` and `run_shell` require approval (unless overridden).
- `aggressive` mode: `write_file` auto-approved, shell still filtered by policy.
- Disable shell execution entirely:

```powershell
.\.venv\Scripts\python.exe tools/atlas_agent/main.py --goal "..." --disable-shell
```

## Dry Run Mode

Dry-run lets you validate planning behavior without executing tools:

```powershell
.\.venv\Scripts\python.exe tools/atlas_agent/main.py `
  --goal "Propose a fix for telemetry API contract drift" `
  --dry-run-tools
```

## Run Artifacts

Each session writes files under:

`tools/atlas_agent/runs/<timestamp_session_id>/`

- `events.jsonl`: step-by-step trace (LLM replies, tool results, final)
- `summary.json`: compact execution summary
- `write_file` tool events include `diff`, `before_hash`, `after_hash`, `changed`

Episodic memory database:

`tools/atlas_agent/memory/episodes.sqlite`

## JSON Action Contract

The model is forced to output this schema at every step:

```json
{
  "thought": "short reasoning",
  "action": {
    "type": "tool_call",
    "tool": "read_file",
    "args": {"path": "atlas_adapter/atlas_http_api.py"}
  }
}
```

Mark objective completed:

```json
{
  "thought": "Objective achieved after validation",
  "action": {
    "type": "objective_done",
    "objective_id": "o2",
    "content": "Endpoint implemented and smoke-tested"
  }
}
```

Or final:

```json
{
  "thought": "done",
  "action": {
    "type": "final",
    "content": "Summary and recommendations"
  }
}
```

## HTTP Endpoints (PUSH `:8791`)

- `POST /api/agent/autonomous/run`
- `GET /api/agent/autonomous/runs/recent?limit=20`
- `GET /api/agent/autonomous/runs/{session_id}`

`POST /api/agent/autonomous/run` request example:

```json
{
  "goal": "Audit dashboard telemetry and patch stale polling",
  "mode": "aggressive",
  "max_steps": 8,
  "dry_run_tools": false,
  "approved_tools": ["write_file"],
  "approval_overrides": {"run_shell": true}
}
```

## One-Command E2E Audit

Run a full runtime + endpoint audit and export JSON evidence to `snapshots/audit`:

```powershell
python scripts/atlas_agent_e2e_audit.py --restart-push
```

Render executive markdown from latest (or explicit) report:

```powershell
python scripts/atlas_agent_audit_to_md.py
python scripts/atlas_agent_audit_to_md.py --report snapshots/audit/atlas_agent_e2e_YYYYMMDD_HHMMSS.json
```

Register periodic Windows task (every 30 min, no restart by default):

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/register_atlas_agent_e2e_task.ps1 -EveryMinutes 30 -RunNow
```
