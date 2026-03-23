# ATLAS-Quant — Informe Estructural Completo
> Para Claude: mapa de arquitectura, flujo de eventos y API de cada módulo.
> Rama activa: `variante/nueva` | Último commit: `6e842ea7e`

---

## 1. Árbol de Módulos

```
atlas_code_quant/               ← paquete raíz (~27 000 líneas)
│
├── atlas_quant_core.py         ← ORQUESTADOR PRINCIPAL (ATLASQuantCore)
├── chart_launcher.py           ← M10: TradingView FREE + Scanner dinámico
├── main.py                     ← entrypoint FastAPI alternativo
│
├── config/
│   └── settings.py             ← TradingConfig (dataclass) + carga de env vars
│
├── pipeline/                   ← datos de mercado en tiempo real
│   ├── tradier_stream.py       ← WebSocket + REST Tradier
│   └── indicators.py           ← CVDCalculator, IVRankCalculator, TechnicalIndicators
│
├── models/                     ← ML
│   ├── regime_classifier.py    ← XGBoost + LSTM → Bull/Bear/Sideways
│   ├── features.py             ← feature engineering
│   ├── signals.py              ← signal dataclasses
│   └── trainer.py              ← entrenamiento incremental
│
├── strategy/
│   ├── signal_generator.py     ← reglas RSI/MACD/CVD/Vol → TradeSignal
│   └── visual_triggers.py      ← validación por OCR (Insta360)
│
├── risk/
│   └── kelly_engine.py         ← Kelly fraccionado × 0.25, circuit breaker
│
├── execution/
│   ├── live_loop.py            ← loop 5s: OCR→quotes→señal→orden
│   ├── signal_executor.py      ← router paper/live → TradierExecution o HID
│   ├── tradier_execution.py    ← REST Tradier, preview=true forzado
│   ├── tradier_controls.py     ← PDT check, account session
│   ├── tradier_pdt_ledger.py   ← registro day-trades
│   ├── kelly_sizer.py          ← KellySizer (wrapper del engine)
│   ├── mode_switcher.py        ← toggle paper ↔ live con confirmación física
│   ├── account_manager.py      ← balance, posiciones, PnL
│   └── portfolio.py            ← posiciones abiertas y métricas
│
├── hardware/
│   ├── camera_interface.py     ← Insta360 RTMP → OpenCV → EasyOCR/YOLO/Qwen
│   ├── control_interface.py    ← HIDController (pyautogui), TradierWebController
│   └── ros2_bridge.py          ← publicación ROS2 (degradado-gracioso)
│
├── healing/
│   ├── self_healing.py         ← SelfHealingOrchestrator, health checks
│   └── error_agent.py          ← SQLite de errores y reparaciones
│
├── calibration/
│   ├── physical_calibration.py ← mapeo de pantalla (YOLO+contours+heurística)
│   ├── voice_feedback.py       ← pyttsx3 español, 20+ mensajes
│   └── test_runner.py          ← N ciclos paper → Sharpe/DD/WR → go/no-go
│
├── production/
│   ├── production_guard.py     ← daily loss 2%, pos 5%, double confirmation
│   ├── telegram_alerts.py      ← cola async, 10+ alert types
│   ├── grafana_dashboard.py    ← Prometheus Gauges + JSON dashboard
│   ├── grafana_pro.py          ← 23 paneles PRO
│   └── live_activation.py      ← protocolo 5 pasos para go-live
│
├── scanner/
│   ├── opportunity_scanner.py  ← OpportunityScannerService (1326 líneas)
│   └── universe_catalog.py     ← ScannerUniverseCatalog: US equities
│
├── learning/
│   ├── rl_env.py               ← QuantTradingEnv (gym.Env)
│   ├── rl_trainer.py           ← PPO/A2C trainer
│   ├── adaptive_policy.py      ← AdaptiveLearningService: bias por símbolo
│   ├── retraining_scheduler.py ← Optuna cada 4-24h
│   └── visual_state_builder.py ← estado RL desde frames Insta360
│
├── journal/
│   ├── service.py              ← TradingJournalService (SQLite)
│   ├── db.py                   ← schema
│   ├── models.py               ← ORM models
│   └── sync.py                 ← sync con Grafana
│
├── monitoring/
│   ├── advanced_monitor.py     ← NormalizedPosition, build_monitor_summary
│   └── strategy_tracker.py     ← tracking por estrategia
│
├── operations/
│   ├── operation_center.py     ← OperationCenter: config + emergency stop
│   ├── alert_dispatcher.py     ← AlertDispatcher (Telegram + WhatsApp)
│   ├── sensor_vision.py        ← integración sensores visuales
│   └── vision_calibration.py  ← calibración visión
│
├── api/
│   ├── main.py                 ← FastAPI app (:8792), 1429 líneas
│   ├── schemas.py              ← OrderRequest, TradierOrderLeg, etc.
│   ├── decorators.py           ← auth, rate limit
│   └── routes/                 ← endpoints REST
│
├── data/
│   ├── feed.py                 ← DataFeed histórico
│   └── realtime_feed.py        ← RealtimeFeed (REST polling)
│
├── backtesting/
│   ├── engine.py               ← motor de backtesting
│   ├── metrics.py              ← Sharpe, Sortino, Calmar
│   ├── reporter.py             ← reportes HTML/MD
│   └── winning_probability.py  ← cálculo de win-rate y Kelly empírico
│
├── selector/
│   └── strategy_selector.py   ← StrategySelector: elige estrategia por régimen
│
├── strategies/
│   ├── base.py                 ← Strategy ABC
│   ├── ma_cross.py             ← MA Crossover
│   └── rl_strategy.py          ← estrategia RL (PPO)
│
└── scripts/
    ├── backtest_m7.py          ← CLI backtest histórico
    └── run_backtest.py         ← runner de backtest
```

**Archivos raíz del repo relacionados con Quant:**
```
immediate_start.py              ← arranque inmediato AHORA (scanner → TV → PAPER)
morning_market_test.py          ← 08:00→16:00 ET (wait + fases)
docker/entrypoint.sh            ← modos: immediate-start | morning-test | core | ...
grafana/dashboards/             ← atlas_pro_2026.json (23 paneles)
grafana/provisioning/           ← datasource atlas-prom-local (:9091)
reports/                        ← auditorías, EOD, scanner snapshots
calibration/atlas_screen_map.json ← mapa de pantalla con scanner_result
```

---

## 2. Orquestador Principal — `ATLASQuantCore`

**Archivo:** `atlas_code_quant/atlas_quant_core.py` (871 líneas)

```python
class ATLASQuantCore:
    # Métodos públicos (todos los entry points):

    setup()                         # inicializa M1→M7 en orden
    start()                         # loop LiveLoop hasta señal de parada
    start_immediate(symbols, skip_tv)           # arranca AHORA sin esperar ET
    start_immediate_dynamic(token, universe, skip_tv)  # scanner → top_symbols → PAPER
    start_full_autonomy(symbols, fullscreen)    # TV FREE → calibración → LiveLoop
    start_market_open_test(skip_wait, now_mode) # delega a morning_market_test.py
    activate_live(symbols, cycles)              # protocolo 5 pasos go-live
    calibrate(force)                            # calibración física de pantalla
    final_audit()                               # auditoría completa M0-M10
    status()                                    # dict con estado de todos los módulos
```

**Orden de inicialización en `setup()`:**
```
M1  hardware.camera_interface    → CameraInterface (RTMP Insta360)
M1  hardware.control_interface   → HIDController(armed=False)
M1  hardware.ros2_bridge         → ATLASRos2Bridge (degradado si sin ROS2)
M2  pipeline.tradier_stream      → TradierStreamClient(sandbox=True)
M2  pipeline.indicators          → CVDCalculator, IVRankCalculator, TechnicalIndicators×N
M3  models.regime_classifier     → RegimeClassifier.load_or_train()
M4  strategy.signal_generator    → SignalGenerator()
M4  strategy.visual_triggers     → VisualTriggerValidator()
M5  risk.kelly_engine            → KellyRiskEngine(initial_capital=100_000)
M6  healing.error_agent          → ErrorMemoryAgent().initialize()
M6  healing.self_healing         → SelfHealingOrchestrator + register_check×3
M7  execution.signal_executor    → SignalExecutor(mode, hid, error_agent)
M7  execution.live_loop          → LiveLoop(todos los módulos, symbols, mode)
    camera.start()               → inicia captura RTMP
```

---

## 3. Flujo de Eventos — Secuencia Completa

### 3A. Arranque Inmediato (HOY — `immediate_start.py`)

```
T+0s   ImmediateStart.run()
         ├── ProductionGuard.force_paper_mode()
         │     ATLAS_MODE=paper, ATLAS_FORCE_LIVE_PREVIEW=true
         │
         ├── _launch_tradingview()          [M10 DINÁMICO]
         │     ├── run_dynamic_scanner(token)
         │     │     ├── GET /markets/quotes  (universo 12 símbolos)
         │     │     ├── GET /options/chains  (IV por símbolo)
         │     │     ├── Filtros: IV Rank>70%, IV/HV>1.2, CVD>+1σ, liq>$10M
         │     │     └── → top_symbols (≤6, ordenados por score compuesto)
         │     └── ChartLauncher.launch_free_tradingview(top_symbols)
         │           ├── subprocess.Popen([chrome, '--new-window', url_sym0])
         │           ├── pyautogui: F11 fullscreen
         │           ├── por cada sym restante: Ctrl+T → Ctrl+L → typewrite(url) → Enter
         │           ├── pyautogui: Alt+5 (timeframe 5m en TV)
         │           └── _save_screen_coords() → calibration/atlas_screen_map.json
         │
         ├── _init_voice()                  VoiceFeedback().start()
         ├── _init_telegram()               TelegramAlerts()
         ├── _init_grafana()                GrafanaDashboard(); _started=True
         ├── _init_calibration()            PhysicalCalibrator.map_exists()
         ├── _announce_start()              speak() + Telegram "ATLAS arrancando AHORA"
         ├── _init_stream()                 TradierStreamClient(sandbox).connect()
         │
         ├── _premarket_loop()              [hasta 09:30 ET]
         │     loop cada 60s:
         │       GET /markets/quotes        quotes REST
         │       _run_scan()                log precios + spreads
         │       GrafanaDashboard.update()  métricas Prometheus
         │
         └── _trading_session()             [09:30 → 16:00 ET]
               ATLASQuantCore(mode='paper').setup()
               live_loop.start()            → inicia loop 5s en background thread
               while not 16:00:
                 _update_grafana_metrics()
                 sleep(5)
               live_loop.stop("market_close")
```

### 3B. Loop de Trading — `LiveLoop._run_cycle()` (cada 5s)

```
Ciclo #N  (cada _CYCLE_S = 5 segundos)
  │
  ├── 1. CPU check
  │       psutil.cpu_percent() > 80% → ocr_skipped=True
  │
  ├── 2. OCR / Visual state   [si CPU<80% y Δt>_OCR_INTERVAL_S]
  │       CameraInterface.latest_result()
  │         ← RTMP frame → EasyOCR/YOLO → precios, patrones
  │       VisualTriggerValidator.validate(ocr_result)
  │
  ├── 3. Indicadores técnicos
  │       TechnicalIndicators[sym].update(bid, ask, vol, ts)
  │       CVDCalculator.update_trade(price, size, ts)   ← from _trade_queue
  │       IVRankCalculator.update_iv(sym, iv)           ← from quote callbacks
  │
  ├── 4. Clasificación de régimen
  │       RegimeClassifier.predict(features)
  │         features = (RSI, MACD, vol_spike, CVD, IV_rank, price_momentum)
  │         → MarketRegime: BULL / BEAR / SIDEWAYS + confidence
  │
  ├── 5. Generación de señal
  │       SignalGenerator.evaluate(sym, regime, tech_snap, cvd_snap, ocr)
  │         reglas: RSI divergencia + MACD cross + vol_spike + CVD>1σ + visual
  │         → TradeSignal(type=BUY_CALL/BUY_PUT/CLOSE, sym, confidence)
  │         ProductionGuard.gate_order(value, portfolio_value)
  │           → bloquea si daily_loss>2% o pos_size>5%
  │
  ├── 6. Sizing Kelly
  │       KellyRiskEngine.compute_size(signal, equity, vol)
  │         Kelly bruto → × 0.25 (quarter-Kelly)
  │         inverse-vol scaling: size ÷ σ_normalizado
  │         → PositionSize(qty, limit_px, dollar_risk)
  │
  ├── 7. Ejecución
  │       SignalExecutor.execute(signal, position_size)
  │         mode = os.getenv("ATLAS_MODE")  ← hot-reload F9
  │         if paper:
  │           route_order_to_tradier(preview=True, sandbox=True)
  │             POST /accounts/{VA9201365}/orders  preview=true
  │             → ExecutionResult(status="ok", cost, day_trades)
  │         if live:
  │           require_double_confirmation()  ← 3×círculo mouse + F12
  │           route_order_to_tradier(preview=False, live=True)
  │         fallback: HIDController.execute_order()  ← pyautogui
  │
  ├── 8. Self-Healing
  │       SelfHealingOrchestrator._check_all()
  │         check "camera"        → ocr_confidence ≥ 0.92
  │         check "tradier_stream"→ _running and reconnect_count < 5
  │         check "risk_engine"   → not circuit_breaker_active
  │         on fail: ErrorMemoryAgent.record_error() → _repair_*()
  │
  ├── 9. Métricas Grafana
  │       GrafanaDashboard.update(equity, drawdown, pnl, open_pos, ...)
  │         Prometheus Gauges → scrapeados por :9091
  │
  └── 10. ROS2 publish (si disponible)
          ATLASRos2Bridge.publish_signal(signal)
          ATLASRos2Bridge.publish_status(cycle_metrics)
```

### 3C. Flujo del Scanner Dinámico — `run_dynamic_scanner()`

```
Entrada: token, universe[12], filtros(iv_rank>70, iv_hv>1.2, cvd>1σ, liq>$10M)
│
├── GET sandbox/v1/markets/quotes?symbols=SPY,QQQ,AAPL,...
│     → quote_map: {sym: {last, bid, ask, bid_sz, ask_sz, vol, hi, lo}}
│
├── para cada sym en universe:
│     GET sandbox/v1/markets/options/expirations
│     GET sandbox/v1/markets/options/chains?greeks=true
│     → iv_map: {sym: avg_ATM_IV}
│
├── para cada sym con quote y iv:
│     liq_usd = vol × last                    → filtro ≥ $10M
│     iv_rank = percentil(iv vs universe_ivs)  → filtro ≥ 70%
│     hv_proxy = (hi-lo)/last × √252          → proxy HV intradiario
│     iv_hv   = iv / hv_proxy                 → filtro ≥ 1.2
│     cvd_raw = (ask_sz - bid_sz) / total_sz  → proxy presión compradora
│     cvd_σ   = cvd_raw / 0.1                → filtro ≥ 1.0σ
│     score   = rank×0.4 + iv_hv×10 + cvd×5 + min(liq/1e8,10)
│
├── ordenar por score DESC → top ≤ 6
├── fallback si 0 resultados: top-4 por liquidez
└── → top_symbols: ['SPY', 'NVDA', 'AMD']  (resultado real 2026-03-23)
```

---

## 4. API de Cada Módulo — Referencia Rápida

### `config/settings.py` — `TradingConfig`
```python
settings.tradier_paper_token    # TRADIER_PAPER_TOKEN env
settings.tradier_live_token     # TRADIER_LIVE_TOKEN env
settings.scanner_universe       # lista de símbolos por defecto
settings.atlas_mode             # no disponible → usar os.getenv("ATLAS_MODE")
_fenv(key, default) → float     # helper para env vars float
_ienv(key, default) → int       # helper para env vars int
```

### `pipeline/tradier_stream.py` — `TradierStreamClient`
```python
client = TradierStreamClient(token, sandbox=True, reconnect_delay_s=3.0)
client.subscribe(["SPY","NVDA"])
client.on_quote(callback: Callable[[StreamQuote], None])
client.on_trade(callback: Callable[[StreamTrade], None])
client.on_error(callback: Callable[[str], None])
client.connect()                # non-blocking, starts 2 threads
client.disconnect()
client.get_quote(sym) → dict    # REST fallback
client.get_options_chain(sym, expiration) → OptionsChain
client.stats() → dict           # running, reconnect_count, queue sizes

# LIMITACIÓN SANDBOX: WebSocket requiere scope stream.scopeSet
# → usar get_quote() REST como fallback en paper mode
```

### `pipeline/indicators.py`
```python
# CVD — Cumulative Volume Delta (Lee-Ready rule)
cvd = CVDCalculator()
cvd.update_quote(bid, ask)
cvd.update_trade(price, size, ts)
snap = cvd.snapshot()           # CVDSnapshot(cvd, cvd_1σ, buying_pct, selling_pct)
cvd.reset()

# IV Rank (percentil 30 días)
iv = IVRankCalculator()
iv.update_iv(sym, iv_value)
metrics = iv.metrics(sym)       # IVMetrics(current_iv, rank, hv_30d, iv_hv_ratio)

# Técnicos
ti = TechnicalIndicators(sym)
ti.update(bid, ask, vol, ts)
snap = ti.snapshot()            # TechnicalSnapshot(rsi, macd, macd_sig, vol_spike, bb_pct)
ti.rsi_divergence() → bool
ti.volume_spike() → bool
```

### `models/regime_classifier.py` — `RegimeClassifier`
```python
clf = RegimeClassifier(use_gpu=True)
clf.load_or_train(symbols=["SPY","QQQ"])  # carga modelo o entrena con histórico
result = clf.classify(symbol="SPY")
# result.regime  → MarketRegime.BULL / BEAR / SIDEWAYS
# result.confidence → float 0-1
# result.features → dict con valores usados
clf.save(path)
```

### `strategy/signal_generator.py` — `SignalGenerator`
```python
gen = SignalGenerator()
signal = gen.evaluate(
    symbol="SPY",
    regime=MarketRegime.BULL,
    tech_snap=TechnicalSnapshot,
    cvd_snap=CVDSnapshot,
    ocr_result=OCRResult,          # puede ser None
)
# signal.type → SignalType.BUY_CALL / BUY_PUT / CLOSE / NEUTRAL
# signal.symbol, signal.confidence, signal.reason
gen.open_positions() → list[OpenPosition]
gen.signal_history  → deque[TradeSignal]
```

### `risk/kelly_engine.py` — `KellyRiskEngine`
```python
kelly = KellyRiskEngine(initial_capital=100_000.0)
pos = kelly.compute_size(signal, current_equity, volatility)
# pos.quantity → int (shares)
# pos.limit_price → float
# pos.dollar_risk → float
kelly.record_trade(pnl, win)
state = kelly.state()
# state.current_equity, state.drawdown_pct, state.kelly_fraction
# state.circuit_breaker_active → True si drawdown >= 10%
kelly.reset_circuit_breaker()
```

### `execution/signal_executor.py` — `SignalExecutor`
```python
exec = SignalExecutor(mode="paper", hid_controller, error_agent)
result = exec.execute(signal, position_size)
# mode re-leído de os.getenv("ATLAS_MODE") en cada llamada → hot-reload F9
# result.status → "ok" | "preview" | "blocked" | "error"
# result.order_id, result.cost, result.day_trades
exec.stats() → dict
```

### `execution/live_loop.py` — `LiveLoop`
```python
loop = LiveLoop(camera, stream, tech_indicators, cvd_calc, iv_rank,
                regime_clf, signal_gen, risk_engine, executor,
                healer, ros2, hid, symbols, mode, cycle_s=5.0)
loop.start()                    # thread background "atlas-live-loop"
loop.join(timeout)              # bloquea hasta stop
loop.stop(reason)               # graceful
loop.emergency_stop()           # inmediato
loop.pause() / loop.resume()
loop.update_symbols(new_syms)   # hot-reload de símbolos del scanner
loop.update_quote(sym, quote)   # feed externo de quote
loop.status() → dict            # cycle_count, mode, running, last_cycle_ms

# Hotkeys (pynput):
# Esc  → emergency_stop()
# F12  → HID arm/disarm
# F10  → forzar ciclo inmediato
# F9   → toggle paper/live
```

### `execution/tradier_execution.py`
```python
# _FORCE_LIVE_PREVIEW = os.getenv("ATLAS_FORCE_LIVE_PREVIEW","true") != "false"
# Siempre True por defecto → ninguna orden real sin override explícito

result = route_order_to_tradier(body: OrderRequest, account_session)
payload = build_tradier_order_payload(body) → dict
# Endpoints:
#   sandbox: POST https://sandbox.tradier.com/v1/accounts/{id}/orders
#   live:    POST https://api.tradier.com/v1/accounts/{id}/orders
# preview=true → Tradier simula y devuelve cost/day_trades sin ejecutar
```

### `hardware/camera_interface.py` — `CameraInterface`
```python
cam = CameraInterface(rtmp_url="rtmp://192.168.1.10/live/atlas", use_gpu=True)
cam.on_degraded(callback)       # callback cuando OCR < 0.92
cam.start()                     # thread de captura RTMP
cam.stop()
result = cam.latest_result()    # OCRResult(prices, patterns, confidence, ts)
prices = cam.get_latest_prices() → dict[str, float]
cam.get_ocr_confidence() → float  # 0-1
cam.avg_latency_ms() → float
```

### `hardware/control_interface.py` — `HIDController`
```python
hid = HIDController(armed=False)
hid.start_hotkey_listener()     # pynput listener
hid.stop_hotkey_listener()
hid.move_to(x, y)               # pyautogui
hid.click(x, y, button="left")
hid.type_text(text)
hid.hotkey(*keys)               # e.g. hotkey("ctrl","t")
hid.execute_order(HIDOrder)     # navega Tradier UI con mouse/teclado
hid.action_history → list[dict]
```

### `production/production_guard.py` — `ProductionGuard`
```python
ProductionGuard.force_paper_mode()          # static: fuerza env vars PAPER

guard = ProductionGuard(voice, hid, daily_loss_pct=2.0, max_position_pct=5.0)
ok, reason = guard.check_ready_for_live()  # lee ready_for_live.json
ok, reason = guard.gate_order(order_value, portfolio_value)
# bloquea si daily_pnl < -2% equity o order_value > 5% portfolio
guard.record_trade_pnl(pnl)
guard.require_double_confirmation()        # 3×círculo mouse + F12 + voz
guard.announce_paper_mode()               # voz + Telegram
guard.status() → dict
```

### `production/telegram_alerts.py` — `TelegramAlerts`
```python
tg = TelegramAlerts()
tg.start()                       # worker thread non-blocking queue(50)
tg.stop()
tg.send(text)                    # encola mensaje
TelegramAlerts.send_static(text) # class method, instancia temporal
# Métodos especializados:
tg.alert_live_activated(equity)
tg.alert_order_executed(sym, side, qty, price, mode)
tg.alert_order_blocked(sym, reason)
tg.alert_drawdown(pct, equity)
tg.alert_circuit_breaker(pct)
tg.alert_emergency_stop()
tg.alert_sharpe(sharpe, mode)
tg.alert_repair_start(subsystem, action)
# Credenciales: TELEGRAM_BOT_TOKEN + TELEGRAM_CHAT_ID (env vars)
```

### `production/grafana_dashboard.py` — `GrafanaDashboard`
```python
g = GrafanaDashboard()
g.start_metrics_server(port=9090)   # Prometheus HTTP exporter
g.update(
    equity=float, drawdown=float, daily_pnl=float,
    open_pos=int, sharpe=float, iv_rank=float,
    ocr_conf=float, cycle_ms=float, cpu_pct=float,
    regime_label="bull"|"bear"|"sideways",
    mode="paper"|"live",
    new_trade=bool
)
g.make_cycle_callback() → Callable  # para pasar a LiveLoop
g.save_dashboard()                  # escribe grafana/dashboards/atlas.json
g.save_pro_dashboard()              # guarda atlas_pro_2026.json (23 paneles)
# Gauges expuestas: atlas_equity_usd, atlas_drawdown_pct, atlas_daily_pnl_usd,
#   atlas_open_positions, atlas_sharpe_ratio, atlas_iv_rank,
#   atlas_ocr_confidence_pct, atlas_cycle_ms, atlas_cpu_pct,
#   atlas_regime (0=Bear,1=Sideways,2=Bull), atlas_mode (0=paper,1=live)
```

### `chart_launcher.py` — `ChartLauncher` + `run_dynamic_scanner()`
```python
# SCANNER DINÁMICO
top = run_dynamic_scanner(
    token=TRADIER_PAPER_TOKEN,
    universe=["SPY","QQQ","AAPL",...],   # default: 12 símbolos
    iv_rank_min=70.0,
    iv_hv_ratio_min=1.2,
    cvd_sigma_min=1.0,
    liq_min_usd=10e6,
    max_symbols=6,
) → ["SPY","NVDA","AMD"]               # resultado real hoy

# CHART LAUNCHER
launcher = ChartLauncher(symbols=top_symbols)
launcher.launch_free_tradingview(fullscreen=True) → bool
launcher.launch_dynamic_tradingview(
    token, universe, fullscreen,
    iv_rank_min, iv_hv_ratio_min, cvd_sigma_min, liq_min_usd
) → list[str]                          # top_symbols abiertos
launcher.switch_to_symbol("NVDA") → bool
launcher.capture_screenshot("SPY") → bytes | None
launcher.close_all()

# URLs generadas:
# Stock:  https://www.tradingview.com/chart/?symbol=NASDAQ%3AAAPL&interval=5
# ETF:    https://www.tradingview.com/chart/?symbol=AMEX%3ASPY&interval=5
# Sin login, sin suscripción, 3 indicadores free: RSI(14)+MACD+Volumen
```

### `healing/self_healing.py` — `SelfHealingOrchestrator`
```python
healer = SelfHealingOrchestrator(memory_agent, alert_callback)
healer.register_check(
    name="camera",
    health_fn=lambda: cam.get_ocr_confidence() >= 0.92,
    error_type="ocr_confidence_low",
    critical=False
)
healer.start()                  # thread background cada 30s
healer.stop()
healer.health_summary() → dict  # {name: HealingStatus}
healer.overall_status() → "healthy"|"degraded"|"critical"
# Reparaciones automáticas:
# _repair_websocket() → reconecta TradierStreamClient
# _repair_api()       → reinicia sesión Tradier
# _repair_camera()    → reinicia CameraInterface
# _repair_ros2()      → reinicia ATLASRos2Bridge
```

### `scanner/opportunity_scanner.py` — `OpportunityScannerService`
```python
# NOTA: módulo scanner/ es más completo que run_dynamic_scanner()
# Úsalo para escaneos más profundos en producción

scanner = OpportunityScannerService()
scanner.update_config(ScannerConfig(
    iv_rank_min=70, iv_hv_min=1.2, cvd_sigma=1.0,
    liq_min_usd=10e6, max_symbols=6
))
candidates = scanner.criteria_catalog()   # lista de candidatos con scores
report = scanner.report()                 # resumen completo
status = scanner.status()
```

---

## 5. Variables de Entorno — Referencia Completa

| Variable | Default | Módulo que la lee | Descripción |
|----------|---------|------------------|-------------|
| `ATLAS_MODE` | `paper` | `signal_executor`, `live_loop`, `mode_switcher` | `paper` ó `live` — hot-reload en cada ciclo |
| `ATLAS_FORCE_LIVE_PREVIEW` | `true` | `tradier_execution` | Fuerza `preview=True` siempre |
| `TRADIER_PAPER_TOKEN` | — | `settings`, `scanner`, `stream` | Token sandbox.tradier.com |
| `TRADIER_LIVE_TOKEN` | — | `settings`, `mode_switcher` | Token api.tradier.com |
| `TRADIER_LIVE_ACCOUNT_ID` | — | `mode_switcher` | Requerido para activar LIVE |
| `TELEGRAM_BOT_TOKEN` | — | `telegram_alerts`, `mode_switcher` | Bot @RauliAtlasBot |
| `TELEGRAM_CHAT_ID` | — | `telegram_alerts` | Chat ID = 1749113793 |
| `ATLAS_SYMBOLS` | `AAPL,TSLA,SPY,QQQ` | `immediate_start`, `docker entrypoint` | Override de símbolos |
| `ATLAS_RTMP_URL` | `rtmp://192.168.1.10/live/atlas` | `camera_interface` | Stream Insta360 |
| `ATLAS_SCREEN_MAP` | `/calibration/atlas_screen_map.json` | `physical_calibration` | Mapa de pantalla |
| `ATLAS_CYCLE_S` | `5.0` | `live_loop` | Segundos entre ciclos |
| `ATLAS_OCR_INTERVAL_S` | `0.5` | `live_loop` | Intervalo OCR mínimo |
| `ATLAS_CPU_THROTTLE_PCT` | `80` | `live_loop` | % CPU para throttle OCR |
| `ATLAS_TTS_LANG` | `es` | `voice_feedback` | Idioma síntesis de voz |
| `ATLAS_DAILY_LOSS_PCT` | `2.0` | `production_guard` | % pérdida diaria máxima |
| `ATLAS_MAX_POS_PCT` | `5.0` | `production_guard` | % portfolio por posición |
| `ATLAS_READY_FILE` | `data/operation/ready_for_live.json` | `production_guard` | Flag go-live |
| `QUANT_KELLY_FRACTION` | `0.25` | `settings` | Multiplicador Kelly |
| `TRADIER_CREDENTIALS_FILE` | — | `settings` | Ruta a credenciales.txt |

---

## 6. Secuencia de Eventos — Diagrama de Tiempo

```
T  COMPONENTE          EVENTO / MÉTODO LLAMADO
── ─────────────────── ──────────────────────────────────────────────────────
0  immediate_start.py  ImmediateStart.run()
0  production_guard    force_paper_mode() → ATLAS_MODE=paper
1  chart_launcher      run_dynamic_scanner(token)
     tradier REST        GET /quotes × 12 símbolos
     tradier REST        GET /options/chains × 12 (IV data)
     scanner             filtros IV Rank/HV/CVD/Liq → top_symbols
3  chart_launcher      launch_free_tradingview(top_symbols)
     subprocess          chrome.exe --new-window URL_sym0
     pyautogui           F11 (fullscreen), Ctrl+T×N, typewrite URLs
     screen_map          calibration/atlas_screen_map.json updated
10 voice_feedback      VoiceFeedback.start()
10 telegram_alerts     TelegramAlerts() init
10 grafana_dashboard   GrafanaDashboard(); _started=True
10 physical_calib      map_exists() → True → load coords
11 telegram            "ATLAS arrancando AHORA" enviado
12 tradier_stream      TradierStreamClient.connect()
                         _create_session() → POST /events/session
                         ws.run_forever(ping_interval=25)
15 immediate_start     _premarket_loop() [hasta 09:30 ET]
     loop cada 60s       GET /quotes REST
                         log precios + spreads
                         GrafanaDashboard.update(mode='paper')
540 immediate_start    09:30 ET → _trading_session()
     atlas_quant_core    ATLASQuantCore.setup()
       M1 hardware        CameraInterface.start()
       M2 pipeline        stream.subscribe(top_symbols)
       M3 régimen         RegimeClassifier.load_or_train()
       M4 señales         SignalGenerator()
       M5 kelly           KellyRiskEngine(100_000)
       M6 healing         SelfHealingOrchestrator.start()
       M7 loop            LiveLoop.start() → thread "atlas-live-loop"
545 live_loop          Ciclo #1: OCR→quotes→régimen→señal→kelly→orden
     camera_interface    latest_result() → precios del chart
     indicators          TechnicalSnapshot(RSI,MACD,vol_spike)
     regime_clf          predict() → BULL/BEAR/SIDEWAYS
     signal_gen          evaluate() → TradeSignal
     production_guard    gate_order() → ok/blocked
     kelly_engine        compute_size() → qty, limit_px
     signal_executor     execute(signal, size)
       tradier_exec        POST /orders preview=True sandbox → status=ok
     grafana             update(equity,drawdown,pnl,open_pos,regime)
     self_healing        _check_all() → todos OK
     ros2_bridge         publish_signal() (si ROS2 disponible)
550 live_loop          Ciclo #2… repeat cada 5s
960 live_loop          16:00 ET → loop.stop("market_close")
961 immediate_start    _eod_report()
     reports/            immediate_eod_20260323.md
     voice               "Sesión terminada. Revisa Grafana."
     telegram            EOD report con equity y PnL
```

---

## 7. Docker — Modos del Entrypoint

**Archivo:** `docker/entrypoint.sh`

| Modo | Comando | Qué hace |
|------|---------|----------|
| `immediate-start` | `python immediate_start.py` | Scanner → TV → PAPER NOW |
| `morning-test` | `python morning_market_test.py` | 08:00→16:00 ET con fases |
| `morning-test` + `SKIP_WAIT=true` | `morning_market_test.py --now` | Fases inmediatas |
| `core` | `atlas_quant_core --mode paper` | Loop puro sin scanner TV |
| `core-live` | `atlas_quant_core --mode live` | Requiere TRADIER_LIVE_TOKEN |
| `calibrate` | PhysicalCalibrator | Solo calibración |
| `test` | TestRunner N ciclos | Evaluación pre-live |
| `audit` | `atlas_quant_core --final-audit` | Auditoría completa |
| `train` | RegimeClassifier._train_from_history | Re-entrena modelo |
| `live-activation` | `live_activation.activate()` | Protocolo 5 pasos go-live |
| `grafana-setup` | GrafanaDashboard.save_*() | Genera JSON dashboard |

**Variables requeridas por `immediate-start`:**
```bash
TRADIER_PAPER_TOKEN=UqYAFhBY...   # obligatorio
ATLAS_MODE=paper                   # siempre
ATLAS_FORCE_LIVE_PREVIEW=true     # siempre
TELEGRAM_BOT_TOKEN=...            # opcional pero recomendado
TELEGRAM_CHAT_ID=1749113793       # opcional
```

---

## 8. Flujo de Activación LIVE (cuando sea el momento)

**Archivo:** `production/live_activation.py` — función `activate()`

```
paso 1  step_calibration()          PhysicalCalibrator.map_exists()
paso 2  step_test_runner()          TestRunner.run(50 ciclos)
                                    → Sharpe≥1.5, DD≤10%, WR≥45%
                                    → escribe data/operation/ready_for_live.json
paso 3  step_production_guard()     ProductionGuard.check_ready_for_live()
paso 4  step_tradier_preview_test() POST /orders preview=True live account
paso 5  step_double_confirmation()  require_double_confirmation()
                                    → voz "Se va a activar modo live"
                                    → 3×círculo mouse (pyautogui)
                                    → wait F12 press (≤30s)
→ OK    step_launch_live()          mode_switcher.activate_live_mode()
                                    → os.environ["ATLAS_MODE"] = "live"
                                    → _audit_log() → logs/mode_switch_audit.jsonl
                                    → Telegram "ATLAS LIVE ACTIVADO"
```

---

## 9. Integraciones Externas

| Servicio | URL / Endpoint | Token | Uso |
|---------|---------------|-------|-----|
| Tradier Sandbox | `sandbox.tradier.com/v1` | `UqYAFhBY...` | Paper trading, quotes, options |
| Tradier Live | `api.tradier.com/v1` | TRADIER_LIVE_TOKEN | Solo en modo LIVE activado |
| Telegram Bot | `api.telegram.org/bot{TOKEN}` | `7956423194:AAG5...` | Alertas y reportes |
| Prometheus | `localhost:9091` | — | Métricas scrapeadas por Grafana |
| Grafana | `localhost:3002` | `admin/atlas2026` | Dashboard 23 paneles |
| TradingView | `tradingview.com/chart/` | Sin token | Gráficos gratuitos (read-only) |
| Insta360 | `rtmp://192.168.1.10/live/atlas` | — | Stream RTMP cámara |
| ROS2 | `rclpy` / topics | — | Bridge robot (degradado si ausente) |

---

## 10. Restricciones y Gotchas Críticos

| # | Restricción | Impacto | Solución |
|---|------------|---------|----------|
| 1 | **WS Tradier sandbox** requiere scope `stream.scopeSet` | Sin quotes en tiempo real via WS en sandbox | Usar `get_quote()` REST polling cada 60s |
| 2 | **TradingView free** máx 3 indicadores por chart | No añadir un 4.º indicador | RSI+MACD+Vol = exactamente 3 |
| 3 | **PDT Rule** $25,000 mínimo para día-trading margin | Con <$25k, los day trades son limitados | Sandbox equity=$95,840 → OK |
| 4 | **pyautogui FAILSAFE** lanza excepción si mouse en esquina | Para el launcher | `pyautogui.FAILSAFE=True` en chart_launcher, `False` en production_guard |
| 5 | **`ATLAS_MODE`** se lee en cada `execute()` | Hot-reload F9 funciona | No cachearlo — siempre `os.getenv()` |
| 6 | **Grafana datasource** `atlas-prom-local` (UID fijo) | Dashboard roto si cambia | No cambiar UID del datasource |
| 7 | **Docker sin GPU** en modo paper | Sin CUDA para LSTM | `RegimeClassifier(use_gpu=False)` en sandbox |
| 8 | **Insta360 sin RTMP** | `CameraInterface` sin frames | Modo degradado: `latest_result()` retorna None, `visual_triggers` omitido |

---

*Generado: 2026-03-23 | Branch: variante/nueva | Commit: 6e842ea7e*
*Total módulos: 35 archivos Python en atlas_code_quant/ | ~27,000 líneas*
