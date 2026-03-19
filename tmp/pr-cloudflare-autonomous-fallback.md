## Problema
La automatización de Cloudflare quedaba en estado ambiguo: podía reportar finalización correcta aun cuando el dominio no estaba delegado a los nameservers de Cloudflare, y el acceso público quedaba roto mientras llegaba la propagación de NS.

Además, RAULI-VISION no tenía el mismo flujo de túnel/autocuración aplicado al dashboard principal de ATLAS.

## Impacto
- Endpoints públicos podían quedar inaccesibles aunque los servicios locales estuvieran sanos.
- Operación manual para diagnosticar DNS/NS y levantar fallback temporal.
- RAULI-VISION sin fallback automático de túnel.

## Causa raíz
1. Falta de validación de delegación NS pública vs NS esperados por Cloudflare.
2. Ausencia de fallback automático a quick tunnel cuando el host nombrado fallaba health-check.
3. Falta de inclusión explícita de `vision.<zona>` en la finalización de túnel + credenciales públicas de Vision.

## Solución aplicada

### 1) Finalización de túnel nombrado robusta
Archivo: `scripts/cloudflare_finalize_named_tunnel.ps1`
- Valida delegación NS con `Resolve-DnsName` y compara contra NS esperados de la zona.
- Muestra advertencias claras cuando la delegación aún no está en Cloudflare.
- Mantiene mejoras de inferencia de `account_id` cuando `/accounts` no lista cuentas.
- Agrega host Vision al flujo de DNS:
  - `vision.<zona>` -> `<tunnel-id>.cfargotunnel.com`
- Agrega Vision al YAML generado:
  - `vision.<zona>` -> `http://127.0.0.1:3000`
- Actualiza credenciales de Vision:
  - `CLOUDFLARE_VISION_TUNNEL_NAME`
  - `ATLAS_VISION_APP_URL`
  - `ATLAS_VISION_PUBLIC_URL`
  - `ATLAS_VISION_PUBLIC_API_URL`

### 2) Fallback/autocuración para RAULI-VISION
Archivo nuevo: `scripts/start_cloudflare_vision_tunnel.ps1`
- Verifica salud de Vision en `/api/health`.
- Si el host nombrado no responde, levanta quick tunnel (`*.trycloudflare.com`) contra `http://127.0.0.1:3000`.
- Persiste URL activa de Vision en credenciales.
- Cuando el host nombrado se recupere, restaura URL estable y apaga quick tunnel de Vision.
- Permite autostart en HKCU Run (`ATLAS_CLOUDFLARED_VISION`).

## Validación ejecutada
- `cloudflare_finalize_named_tunnel.ps1` ejecutado con token real: creó DNS `vision.midominio.com` y actualizó config/credenciales.
- `start_cloudflare_vision_tunnel.ps1` ejecutado con fallback activo.
- Health checks verificados:
  - Local Vision: `http://127.0.0.1:3000/api/health` -> `200`
  - Público Vision quick tunnel -> `200`
- RunKey de autoarranque Vision confirmado en `HKCU\Software\Microsoft\Windows\CurrentVersion\Run`.

## Estado operativo actual
- El dominio sigue con NS del registrador anterior (`interwebserver.com`), por lo que los host nombrados aún no enrutan por Cloudflare.
- El fallback quick tunnel mantiene ATLAS y RAULI-VISION accesibles públicamente hasta completar delegación NS.
