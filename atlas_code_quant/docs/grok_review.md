# Grok Advisory Review for Options Pipeline

ATLAS Code Quant puede consultar Grok (xAI API) como revisor experto de trades de opciones en modo advisory / soft-veto solo para `paper`.

## Feature flag

- `QUANT_GROK_REVIEW_ENABLED`
  - `true`, `1`, `yes`, `on` -> habilita Grok
  - `false`, `0`, `no`, `off` -> deshabilita Grok

Default actual:

- `disabled` si la variable no existe

Ese default es deliberadamente conservador para preservar un comportamiento seguro por defecto. Si Grok no se habilita de forma explícita, el pipeline sigue funcionando sin llamadas externas.

## Variables de entorno

- `QUANT_GROK_REVIEW_ENABLED`
- `XAI_API_KEY`
- opcionales:
  - `XAI_API_URL`
  - `XAI_MODEL`
  - `XAI_TIMEOUT_SECONDS`
  - `XAI_MAX_RETRIES`
  - `XAI_BACKOFF_BASE_SECONDS`
  - `XAI_TEMPERATURE`
  - `XAI_MAX_TOKENS`

## Cómo activar Grok en una sesión

PowerShell:

```powershell
$env:QUANT_GROK_REVIEW_ENABLED = "true"
$env:XAI_API_KEY = "..."
```

## Cómo desactivar Grok en una sesión

PowerShell:

```powershell
$env:QUANT_GROK_REVIEW_ENABLED = "false"
```

## Comportamiento de fallback

Si Grok está deshabilitado, falta la API key, falla la red, se agotan los retries o la respuesta no parsea:

- el pipeline continúa
- el review vuelve como `neutral`
- no se bloquea la ejecución por fallo técnico de Grok
- se registra `grok_review` con `status=fallback`

## Alcance operativo

- advisory / soft-veto en `paper`
- puede:
  - bloquear una entrada si `verdict=reject`
  - ajustar score de forma suave
  - reducir contratos con `contracts_multiplier`
- no controla directamente `create_position`
- no toca `AutoCloseEngine`
- no habilita `live`
