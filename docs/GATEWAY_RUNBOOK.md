# Stealth Gateway — Runbook

Conectividad segura multi-vía (Cloudflare Tunnel / Tailscale / SSH reverse / LAN) para Atlas Cluster. No exponer puertos públicamente.

## 1. Configuración mínima (atlas.env)

- `GATEWAY_ENABLED=true`
- `GATEWAY_MODE=auto` (o `cloudflare` | `tailscale` | `ssh` | `lan`)
- `GATEWAY_HEALTH_MIN_SCORE=75`
- `GATEWAY_BOOTSTRAP_RETRY_SECONDS=5`
- `GATEWAY_BOOTSTRAP_MAX_RETRIES=10`

## 2. Cloudflare Tunnel

- Descargar `cloudflared` y colocar en `C:\ATLAS_PUSH\bin\cloudflared.exe` (o `CLOUDFLARE_CLOUDFLARED_PATH`).
- **Token mode:** `CLOUDFLARE_TOKEN=<token>` (no guardar en texto si privacy strict). Ejecutar: `cloudflared tunnel run --token <token>`.
- **Named tunnel:** configurar tunnel en Zero Trust dashboard; `CLOUDFLARE_TUNNEL_NAME=atlas-worker-1` y opcionalmente `CLOUDFLARE_TUNNEL_URL=https://...`.
- Script: `scripts\gateway\11_gateway_setup_cloudflare.ps1` (comprueba ruta; no hace login web).

## 3. Tailscale

- Instalar Tailscale. `TAILSCALE_EXE_PATH` si no está en ruta por defecto.
- `TAILSCALE_EXPECTED_NODE=atlas-worker-1`, `TAILSCALE_DNS_NAME=atlas-worker-1`.
- Worker URL: `http://<TAILSCALE_DNS_NAME>:8791`.
- Script: `scripts\gateway\12_gateway_setup_tailscale.ps1`.

## 4. SSH reverse tunnel

- OpenSSH Client instalado. `SSH_USER=atlas`, `SSH_HOST=<HQ host>`, `SSH_REMOTE_PORT=18791`, `SSH_LOCAL_PORT=8791`.
- Usar claves SSH: `ssh-keygen`, copiar clave pública al HQ.
- En worker: `ssh -N -R 18791:127.0.0.1:8791 atlas@<HQ>` (o usar script que llame al wrapper).
- Script: `scripts\gateway\13_gateway_setup_ssh.ps1`.

## 5. LAN

- `LAN_ENABLED=true`, `LAN_WORKER_URL=http://192.168.1.50:8791`.

## 6. Modo auto

Orden de intento: cloudflare → tailscale → ssh → lan. Si uno falla, se prueba el siguiente. Fallbacks automáticos.

## 7. Bootstrap

- HQ descubre workers y valida salud (ping + /health con score ≥ `GATEWAY_HEALTH_MIN_SCORE`).
- Registrar worker en HQ: `POST /cluster/node/register` con `base_url` resultante.
- API: `POST /gateway/bootstrap` (policy `gateway_bootstrap`, solo owner).
- Script: `scripts\gateway\14_gateway_bootstrap.ps1 -RepoPath C:\ATLAS_PUSH`.

## 8. Seguridad

- Policy: `gateway_check`, `gateway_bootstrap`, `gateway_set_mode` — deny by default, allow owner.
- Audit: gateway_start, gateway_fail, gateway_success, bootstrap_attempt/result.
- No imprimir tokens en logs/memory (redacción en audit).

## 9. Endpoints

- `GET /gateway/status` — modo, herramientas, último éxito, recomendaciones.
- `POST /gateway/check` — comprobar candidatos (policy).
- `POST /gateway/bootstrap` — ejecutar bootstrap (policy).
- `POST /gateway/mode` — body `{mode: "auto"|"cloudflare"|...}` (policy).

## 10. UI

En `/ui`, tab Gateway: modo actual, herramientas detectadas, último éxito, botones Check / Bootstrap / Set mode, recomendaciones si falla.
