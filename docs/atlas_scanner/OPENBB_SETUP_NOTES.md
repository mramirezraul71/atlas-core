# OpenBB Setup Notes (ATLAS Scanner)

## Objective

Configure a repeatable OpenBB environment so ATLAS scanner experiments can run against a real provider backend.

This document covers only environment setup and sanity validation. It does not modify scanner code.

## Environment Baseline (this machine)

- OS: Windows 10
- Python: 3.11.x
- Environment type: no active `venv` or `conda` detected

If you use a virtual environment in another machine, run all commands from that environment instead of global/user Python.

## 1) Install OpenBB

### Recommended (isolated venv)

Create and activate a dedicated virtual environment:

```bash
python -m venv .venv_openbb
```

Windows PowerShell:

```powershell
& ".venv_openbb/Scripts/Activate.ps1"
```

Install OpenBB in that environment:

```bash
python -m pip install --upgrade pip
python -m pip install openbb
```

### Alternative (no active venv)

If a venv is not available, install to the user site:

```bash
python -m pip install --user openbb
```

Quick import check:

```bash
python -c "from openbb import obb; print('openbb ok')"
```

## 2) Local OpenBB settings location

OpenBB stores local settings under:

- `~/.openbb_platform/user_settings.json`
- `~/.openbb_platform/.env`

On Windows, this resolves to:

- `%USERPROFILE%\\.openbb_platform\\user_settings.json`
- `%USERPROFILE%\\.openbb_platform\\.env`

## 3) Configure API keys / credentials

OpenBB docs indicate two supported paths:

1. `user_settings.json` (`credentials` object)
2. `.env` inside `~/.openbb_platform`

Example `.env` entries:

```env
FMP_API_KEY="replace_with_real_key"
POLYGON_API_KEY="replace_with_real_key"
BENZINGA_API_KEY="replace_with_real_key"
```

Notes:

- Do not commit keys to this repository.
- Keys in `.env` ending with `_API_KEY` are mapped by OpenBB to credentials.
- Some basic data can work without keys, but coverage varies by provider/endpoint.

## 4) Optional: OpenBB account login flow

If you use OpenBB Hub, you can login and persist a session from Python:

```python
from openbb import obb
obb.account.login(token="YOUR_OPENBB_PAT", keep_session=True)
```

This is optional for basic checks, but useful when backend authorization is required.

## 5) Sanity check script

Run:

```bash
python scripts/openbb_sanity_check.py
```

If using the dedicated venv, run with:

```powershell
& ".venv_openbb/Scripts/python" "scripts/openbb_sanity_check.py"
```

Expected output:

- OpenBB import status
- SPY and QQQ historical query status
- row counts and date range when data is returned

If it fails, the script classifies probable causes:

- `no_backend` (OpenBB import/runtime unavailable)
- `missing_credentials` (auth/key related)
- `parameter_or_endpoint_error` (invalid endpoint/args/provider response)
- `connection_or_runtime_error` (network/runtime issue)

## 6) Known limitations

- OpenBB package combinations in a global Python environment can be inconsistent due to unrelated dependency drift. Prefer a dedicated venv.
- This setup is for scanner diagnostics and offline enrichment checks; it is not a low-latency live feed path.
- OpenBB belongs to the OFFLINE lane for ATLAS (historical/backfill/enrichment), not the real-time live scanner route.

## 7) Sources consulted

- [OpenBB API Keys & Credentials](https://docs.openbb.co/platform/settings/user_settings/api_keys)
- [OpenBB Environment Variables](https://docs.openbb.co/odp/python/settings/environment_variables)
