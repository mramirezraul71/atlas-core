# ATLAS Pending Backlog (2026-03-04)

## Objetivo
Guardar pendientes de `atlas_comms_hub` para ejecutar en la siguiente sesion sin perder contexto.

## Estado actual (ya hecho)
- `atlas_comms_hub` implementado y activo.
- Endpoints `api/comms/atlas/*` en produccion local.
- Cola offline + resync + auditoria funcionando.
- Dashboard v4 muestra cola offline + historial de interacciones.

## Pendientes criticos
1. Configurar token central real para cifrado fuerte:
   - `ATLAS_CENTRAL_CORE=<TOKEN_REAL>`
   - Validar que `GET /api/comms/atlas/status` muestre `encryption_source=ATLAS_CENTRAL_CORE`.

2. Activar alerta urgente via Cloudflare:
   - `ATLAS_CLOUDFLARE_ALERT_URL=<WEBHOOK_REAL>`
   - Probar mensaje urgente y confirmar registro `ATLAS_COMMS_URGENT_ALERT` en `logs/snapshot_safe_diagnostic.log`.

3. Activar sincronizacion remota de cola offline (opcional recomendado):
   - `ATLAS_CLOUDFLARE_SYNC_URL=<SYNC_ENDPOINT_REAL>`
   - Simular offline -> encolar -> reconexion -> `POST /api/comms/atlas/resync`.

4. Activar proveedor ClawdBOT AI real (respuesta hibrida):
   - `ATLAS_CLAWD_API_URL=<ENDPOINT_REAL>`
   - `ATLAS_CLAWD_API_KEY=<API_KEY_REAL>` (o usar `ATLAS_CENTRAL_CORE` si aplica).
   - Verificar que `provider` cambie de `offline_fallback` a `clawd_api`.

5. Validar ajuste de inventario end-to-end con confirmacion:
   - Requiere credencial de panaderia: `ATLAS_PANADERIA_AUTH_BEARER`.
   - Probar `POST /api/comms/atlas/inventory/adjust` con `confirmed=true`.
   - Confirmar que pasa por `atlas_snapshot_safe.ps1` y solo ejecuta si estado estable.

## Checklist de ejecucion rapida (siguiente sesion)
1. Exportar variables de entorno pendientes.
2. Reiniciar API PUSH en `:8791`.
3. Ejecutar:
   - `powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\atlas_comms_init.ps1`
   - `powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\atlas_resync_flow.ps1`
4. Correr smoke API:
   - `GET /api/comms/atlas/status`
   - `POST /api/comms/atlas/message`
   - `GET /api/comms/atlas/history`
   - `POST /api/comms/atlas/resync`
   - `POST /api/comms/atlas/inventory/adjust`

## Nota operativa
- Si falta algun secreto, el sistema sigue operativo en modo fallback.
- No ejecutar acciones destructivas ni rollback global; usar `.atlas_backups` + git selectivo.
