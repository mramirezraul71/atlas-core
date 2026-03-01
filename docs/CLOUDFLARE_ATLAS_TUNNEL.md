# Cloudflare Tunnel para ATLAS

## Objetivo
Exponer `rauli-panaderia` y `RAULI-VISION` mediante Cloudflare Tunnel, manteniendo a ATLAS como punto central de control.

## Artefactos generados
- Config base solicitada: `cloudflare_atlas_config.yaml`
- Config operativa local: `config/cloudflared/cloudflare_atlas_config.yaml`
- Script de preparacion (sin iniciar tunel): `scripts/deploy_cloudflare_atlas.ps1`
- Worker proxy para descargas: `cloudflare/worker/atlas_cuba_proxy_worker.js`
- Wrangler template: `cloudflare/worker/wrangler.toml`
- Guard de IP Cloudflare: `tools/atlas_cloudflare_guard.py`

## Preparacion (sin iniciar tunel)
```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\deploy_cloudflare_atlas.ps1 `
  -TunnelId <TU_TUNNEL_UUID> `
  -CloudflareZone <tu-zona-dns> `
  -SkipInstall
```

> Nota: el script no ejecuta `cloudflared tunnel run`. Solo prepara archivos y valida sintaxis si `cloudflared.exe` existe.

## Subdominios definidos
- `panaderia.<zona>` -> `http://127.0.0.1:5173`
- `panaderia-api.<zona>` -> `http://127.0.0.1:3001`
- `vision.<zona>` -> `http://127.0.0.1:3000`
- `atlas-dashboard.<zona>` -> `http://127.0.0.1:8791`

## QUIC
La configuracion del tunel incluye:
```yaml
protocol: quic
```

## Proxy inverso para descargas (Cuba)
Worker incluido:
- Recibe `?url=<destino>`
- Aplica allowlist por host (`ALLOWED_HOSTS`)
- Reenvia peticion y devuelve respuesta

Variables:
- `ALLOWED_HOSTS` (CSV), por ejemplo:
  - `github.com,registry.npmjs.org,files.pythonhosted.org`

## Seguridad: validacion de IP Cloudflare
Funciones:
- `refresh_cloudflare_ranges()`
- `is_cloudflare_edge_ip(ip)`
- `authorize_dashboard_request(remote_ip, headers)`

Uso rapido:
```powershell
python scripts/atlas_cloudflare_check_ip.py --refresh --ip 1.1.1.1
```

## Auditoria en snapshot seguro
`scripts/atlas_snapshot_safe.ps1` ahora registra:
- estado de proceso `cloudflared`
- estado de tarea `ATLAS_SnapshotSafe` (`TASK_OK ...`)
- ultimas lineas de `logs/cloudflared_access.log` como `CF_TUNNEL_ACCESS ...`

Esto permite auditar intentos de acceso al tunel en `logs/snapshot_safe_diagnostic.log` sin interrumpir la tarea 4h.
