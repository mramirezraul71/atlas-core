# ATLAS Fault Manager v1

## Purpose

Design a native ATLAS system for autodetection, classification, correction, validation, rollback, and structured journaling without moving the current repo structure and without leaving any domain outside the control plane.

This design is inspired by:

- Autoware Universe: system monitor, hazard engine, emergency handler
- ROS diagnostics: standard publishers and aggregation
- ros2_medkit: persistent fault memory and snapshots
- BehaviorTree/Nav2: reactive recovery and fallback validation
- PX4 / ArduPilot: interlocks, failsafes, safe-mode transitions

## Design Principle

ATLAS does not need a foreign robotics stack to replace its current architecture.
ATLAS needs a transversal fault plane that:

1. discovers the whole repo,
2. sees all subsystems,
3. classifies severity consistently,
4. executes bounded recovery playbooks,
5. validates the result,
6. degrades safely when needed,
7. records every step in Bitacora and Telegram.

## What already exists in ATLAS

ATLAS already has useful pieces that should be unified instead of replaced:

- `autonomous/health_monitor`
- `autonomous/self_healing`
- `autonomous/telemetry`
- `modules/humanoid/ans/checks`
- `modules/humanoid/healing`
- `modules/humanoid/watchdog`
- `modules/humanoid/supervisor`
- `atlas_adapter/supervisor_daemon.py`
- `scripts/atlas_autodiagnostic.py`
- `scripts/atlas_self_healing_loop.py`
- `scripts/atlas_sentinel_loop.py`

The correct path is to place a common fault model above these components.

## Scope

The system must cover the whole repo, including at least:

- gateway core
- cognitive core
- robotics core
- workspace core
- autonomy core
- memory core
- operations core
- integration core
- quant core
- runtime, state, scripts, tools, backups, tmp, generated artifacts

This is why the first technical asset is a registry of domains and signal files.

## Architecture

### 1. Fault Registry

Purpose:

- discover top-level domains,
- count files and subtrees,
- identify health/healing/watchdog/supervisor files,
- create a machine-readable map of the entire repo.

Status:

- implemented via `scripts/atlas_fault_registry.py`
- output:
  - `state/atlas_fault_registry.json`
  - `state/atlas_fault_registry_summary.json`

### 2. Diagnostic Layer

Inspired by ROS diagnostics and Autoware system monitor.

Responsibilities:

- run probes for services, ports, endpoints, files, queues, SQLite stores, cameras, model availability, disk, memory, websocket activity, and stale states,
- emit normalized diagnostic events.

Event shape:

- `component_id`
- `domain`
- `severity`
- `fault_code`
- `symptom`
- `evidence`
- `recoverability`
- `first_seen_at`
- `last_seen_at`

### 3. Hazard Engine

Inspired by Autoware `system_error_monitor`.

Responsibilities:

- aggregate local diagnostics,
- compute severity by component, domain, and global system,
- identify propagation chains,
- decide whether the system is:
  - `nominal`
  - `degraded_local`
  - `degraded_system`
  - `manual_assist`
  - `safe_hold`
  - `maintenance`
  - `emergency`

### 4. Recovery Orchestrator

Inspired by Behavior Trees, Nav2 recoveries, and ATLAS self-healing.

Responsibilities:

- choose bounded recovery playbooks,
- limit retries,
- validate post-fix health,
- escalate to fallback or rollback,
- never loop infinitely.

Examples:

- restart local service
- reconnect dependency
- reinitialize model
- reset websocket bridge
- clear stale cache
- disable risky feature
- force degraded mode
- rollback to last known good config

### 5. Safe Mode Manager

Inspired by PX4 / ArduPilot.

Responsibilities:

- enforce non-negotiable interlocks,
- block dangerous operations when prerequisites fail,
- transition to minimum-risk modes.

Examples:

- no actuator control if robot control health is bad
- no sensitive writes if memory integrity is degraded
- no aggressive autonomy if gateway/event bus is unstable
- no critical execution if supervisor validation fails

### 6. Fault Journal

Inspired by ros2_medkit.

Responsibilities:

- persist incidents,
- store attempts and outcomes,
- keep snapshots of evidence,
- remain queryable from UI, audit, and Telegram summaries.

Primary channels:

- Bitacora
- Telegram for warning/critical escalation
- event bus where applicable

## Fault Lifecycle

1. Detection
2. Classification
3. Local recovery attempt
4. Validation
5. Rollback or degrade
6. Structured record

## Current Implementation Order

### Phase 1

- build fault registry
- verify observability channels
- map existing health/healing components

### Phase 2

- normalize diagnostics from current checkers
- define severity model
- define domain and component identifiers

### Phase 3

- add bounded recovery orchestrator over current healing systems
- bind recovery attempts to Bitacora / Telegram

### Phase 4

- add safe-mode manager and interlocks
- validate escalation paths and rollback policies

## Non-goals

- moving repo structure
- replacing all current monitors
- rewriting ATLAS as ROS 2 end-to-end
- uploading or publishing unfinished work in `atlas_code_quant`

## Immediate Deliverables

- `scripts/atlas_fault_registry.py`
- `state/atlas_fault_registry.json`
- `state/atlas_fault_registry_summary.json`
- `contracts/atlas_fault_event.schema.json`
- `scripts/atlas_fault_snapshot.py`
- `state/atlas_fault_snapshot.json`
- `state/atlas_fault_events_latest.json`
- `scripts/atlas_fault_playbooks.py`
- `state/atlas_fault_playbook_state.json`
- `scripts/atlas_fault_manager.py`
- `state/atlas_fault_manager_report.json`
- `state/atlas_fault_manager_latest.json`

These are the seed assets for the ATLAS Fault Manager.

## Verified Runtime Status

Verified on `2026-04-03` in the live repo:

- `atlas_fault_registry.py` scanned the repo and produced:
  - `81` domains
  - `7051` files
  - `454` health/fault/watchdog/supervisor signals
- `atlas_fault_snapshot.py` normalized current diagnostics into the common event contract
- `atlas_fault_manager.py` executed in both:
  - `detect-only`
  - `heal-safe`

Observed result:

- pre-snapshot: `2` degraded events
- post-snapshot: `1` degraded event
- resolved during validation window: `1`
- actionable safe-heal candidates in the current state: `0`

This means the manager is already functional for:

- full-repo discovery
- normalized diagnostic collection
- structured report generation
- comparison between before/after snapshots
- Bitacora journaling of manager cycles

## What Is Functional Right Now

### Functional

- Repo-wide fault registry
- Common fault-event schema
- Diagnostic normalization across existing ATLAS checks and health monitors
- Bounded recovery playbooks by domain / fault code
- Fault manager execution with:
  - pre-check snapshot
  - candidate-heal evaluation
  - playbook planning and execution
  - disruptive-playbook policy gate
  - optional ANS safe-heal phase
  - post-check validation
  - structured report writing
  - Bitacora integration

### Implemented And Verified Separately

- Telegram escalation path from the fault manager

Verified facts:

- credentials are loaded from the local vault (`C:\dev\credenciales.txt`)
- `telegram_bridge` health check returns `ok=True`
- `ops_bus.status()` resolves the effective Telegram chat id
- direct bridge selftest succeeded
- `modules/humanoid/notify.py` selftest also succeeded after loading the vault before resolving the cached chat id
- `atlas_fault_playbooks.py` selftest executed correctly:
  - `activate_fallback` succeeded
  - `enter_degraded_mode` succeeded
  - `exit_degraded_mode` restored `SurvivalMode` to inactive
- `heal-safe` now blocks disruptive playbooks by default:
  - `supervisor.not_running -> restart_service(push)` is skipped unless explicitly enabled
  - non-disruptive playbooks can still run during the same cycle
  - latest verified run executed `comms.whatsapp` remediation and blocked the disruptive `supervisor.daemon` restart by policy

Note:

- the validated fault-manager cycles in this phase did not emit Telegram because the observed events were only `degraded`, not `critical/emergency`
- Telegram is therefore operational, but not exercised by a live critical fault in this verification pass

## Operational Commands

Detect only:

```powershell
python scripts\atlas_fault_manager.py --mode detect-only
```

Safe heal:

```powershell
python scripts\atlas_fault_manager.py --mode heal-safe
```

Safe heal including disruptive playbooks:

```powershell
python scripts\atlas_fault_manager.py --mode heal-safe --allow-disruptive-playbooks
```

JSON summary:

```powershell
python scripts\atlas_fault_manager.py --mode heal-safe --json
```

Playbook selftest:

```powershell
python scripts\atlas_fault_playbooks.py
```
