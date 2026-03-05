# atlas_clawd_bridge

Bridge for ClawdBOT AI to interact safely with ATLAS runtime.

Features:
- Writes evidence into central log: `logs/snapshot_safe_diagnostic.log`
- Enforces stability gate via `scripts/atlas_snapshot_safe.ps1`
- Restricts repo actions to:
  - `_external/rauli-panaderia`
  - `_external/RAULI-VISION`
- Uses `ATLAS_CENTRAL_CORE` (fallback `APPROVALS_CHAIN_SECRET`) for authorization

## CLI quick use

```powershell
python -m tools.atlas_clawd_bridge.bridge status --run-snapshot true --json
python -m tools.atlas_clawd_bridge.bridge evidence --message "bridge online" --json
python -m tools.atlas_clawd_bridge.bridge init --json
```

## HTTP endpoints (PUSH :8791)

- `GET /api/clawd/bridge/status`
- `POST /api/clawd/bridge/evidence`
- `POST /api/clawd/bridge/action`

Use header `X-Atlas-Core: <ATLAS_CENTRAL_CORE>` for authenticated repo actions.

## Immediate activation

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/atlas_agent_init.ps1
```
