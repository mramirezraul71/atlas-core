# atlas_radar_kalshi

Módulo autónomo de **Atlas Core** dedicado al escaneo y trading
sobre los mercados de eventos de **Kalshi v2**. Vive en
`atlas-core/modules/atlas_radar_kalshi/` y opera de forma
independiente respecto a `atlas_push`, compartiendo únicamente las
utilidades base del paquete `core/` (logging, scheduler) y el hook
local de Ollama.

## Arquitectura (4 capas)

```
┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│   Scanner    │ → │    Brain     │ → │   Risk       │ → │   Executor   │
│ WS+REST      │   │ Ollama+      │   │ Kelly +      │   │ Kalshi       │
│ Kalshi v2    │   │ Markov+MC    │   │ caps         │   │ orders       │
└──────────────┘   └──────────────┘   └──────────────┘   └──────────────┘
        │                  │                  │                  │
        └──────── RadarState (in-memory) ─────────────────────────┘
                              │
                              ▼
              FastAPI router /ui/radar (puerto 8791)
              + WebSocket /api/radar/stream
```

* **Scanner** — `wss://...trade-api/ws/v2`, suscribe a
  `orderbook_delta` y `ticker`; un poller REST descubre nuevos
  mercados cada `RADAR_POLL_SECONDS`.
* **Brain** — combina el modelo local de Ollama (sentimiento +
  rationale en JSON), una **Cadena de Markov** sobre buckets de
  precio y una simulación **Monte Carlo** para validar `p(YES)`.
* **Risk** — Criterio de Kelly fraccionario:
  `f* = (p(b+1) - 1) / b`  con `b = (100 - c) / c` y un cap
  `RADAR_MAX_POSITION_PCT`.
* **Executor** — REST autenticado con RSA-PSS (`KalshiSigner`); usa
  `kalshi-python` como SDK oficial y guarda cada operación en
  `logs/radar_orders.jsonl` para auditoría.

## Estructura

```
modules/atlas_radar_kalshi/
├── __init__.py
├── config.py        # Settings tipados (Pydantic) + carga RSA / API key
├── scanner.py       # WebSockets + REST polling
├── brain.py         # Ollama + Markov + Monte Carlo
├── risk.py          # Kelly Criterion
├── executor.py      # Kalshi orders REST
├── main.py          # entry point (CLI + register(app) en atlas-core)
├── requirements.txt
├── .env.example
├── utils/
│   ├── logger.py    # logger compatible con el dashboard de atlas-core
│   └── signer.py    # RSA-PSS (KALSHI-ACCESS-SIGNATURE)
└── dashboard/
    └── router.py    # FastAPI APIRouter + UI dark + WS live stream
```

## Integración con el dashboard central (puerto 8791)

En `atlas_adapter/atlas_http_api.py`:

```python
from modules.atlas_radar_kalshi.main import register
register(app)          # monta /ui/radar, /api/radar/* y arranca el loop
```

* Acceso directo: `http://127.0.0.1:8791/ui/radar`
* Cross-link: el header del UI enlaza con `/`, `/status` y `/modules`
  del adapter; `GET /api/radar/atlas-link` devuelve los URLs en JSON
  para que cualquier widget de atlas-core los consuma.

Modo **stand-alone** (proceso aparte, útil para desarrollo):

```bash
python -m modules.atlas_radar_kalshi.main --serve --port 8792
```

## Variables de entorno

Ver `.env.example`. Mínimo necesario:

```
KALSHI_API_KEY=...
KALSHI_PRIVATE_KEY_PATH=C:\ATLAS\config\kalshi_private.pem
OLLAMA_ENDPOINT=http://127.0.0.1:11434
```

## Instalación aislada

```powershell
.\.venv\Scripts\Activate.ps1
pip install -r modules/atlas_radar_kalshi/requirements.txt
```

## Reglas de aislamiento

* Nada en `atlas_radar_kalshi` importa `brain_core`,
  `mission_manager`, `safety_kernel`, `state_bus`,
  `arbitration`, `policy_store` ni `modules.command_router`.
* Comunicación con el resto de Atlas vía:
  * eventos JSON en el WebSocket `/api/radar/stream`,
  * endpoints REST `/api/radar/*`,
  * archivos de log compartidos (`<log_dir>/atlas.log`).
