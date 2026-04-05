# ATLAS Vision Sensor Architecture

## Purpose
`atlas_vision_sensor` is an out-of-band perception sensor for ATLAS. It captures the TradingView scene through an Insta360 Link 2, extracts structured visual evidence, fuses it with local ATLAS telemetry, and publishes auditable events to ATLAS PUSH on `:8791`.

It does not decide trades and it does not execute orders.

## Existing Infrastructure Reused
- `atlas_adapter/atlas_http_api.py`
  - HTTP ingress to ATLAS PUSH, including `POST /events`
- `modules/humanoid/core/event_bus.py`
  - in-process event bus used by the adapter
- `atlas_code_quant/vision/chart_ocr.py`
  - existing OCR and chart bias extraction logic
- `atlas_code_quant/operations/sensor_vision.py`
  - higher-level Quant bridge/state already used by Code Quant
- `atlas_code_quant/data/operation/operation_center_state.json`
  - local operational telemetry snapshot for Quant

## Modules
- `camera.py`
  - UVC capture, reconnection, offline detection
- `preprocessor.py`
  - resize, grayscale, normalization
- `watchdog.py`
  - heartbeat, freeze detection, degraded mode
- `security.py`
  - human presence detection and privacy mask
- `gesture_detector.py`
  - MediaPipe right-hand emergency gesture
- `pattern_analyzer.py`
  - orchestrates OCR, candlestick detection, geometry detection
- `candle_detector.py`
  - TA-Lib candlestick detection from OHLC telemetry when available
- `geometry_detector.py`
  - channel, triangle, H&S, regression heuristics with temporal consistency
- `multimodal_fusion.py`
  - combines visual evidence with broker/market/system telemetry
- `forensics.py`
  - ring buffer and forensic artifact capture
- `event_publisher.py`
  - structured event delivery to `POST /events` with local queue fallback
- `main.py`
  - lifecycle orchestration

## Data Flow
1. Camera captures a frame.
2. Preprocessor creates a lightweight analysis frame.
3. Watchdog, security, and gesture detector consume raw frames asynchronously.
4. Pattern analyzer builds visual evidence:
   - OCR/chart bias
   - TA-Lib candlestick evidence when OHLC exists
   - geometry heuristics
5. Multimodal fusion adds:
   - system health from PUSH
   - broker/Quant status from local file or local API
   - market context hints from Quant if available
6. Fusion emits an auditable visual signal with:
   - confidence score
   - agreement count
   - contradictions
   - primary source
   - explanation
7. Forensics decides whether to persist a clip and metadata.
8. Event publisher sends structured events to ATLAS PUSH.

## Concurrency and Backpressure
- Camera access remains thread-safe and single-writer.
- Background threads:
  - watchdog
  - security
  - gesture
  - event publisher
- The main loop owns analysis and fusion.
- Queues are bounded.
- When the publisher queue is full or the bus is unavailable, events are persisted to `runtime/vision_sensor_queue/pending_events.jsonl`.
- Analysis prefers fresh frames over exhaustive backlog processing.

## Safety Rules
- Vision never executes orders.
- Confidence is only elevated when at least two modalities agree.
- Missing OHLC does not fabricate TA-Lib output; it emits explicit unavailability.
- Broker or system contradictions reduce confidence and can force a degraded recommendation.
- Emergency gesture stays low-latency but still requires:
  - right hand
  - stable recognition
  - minimum confidence

## Latency Targets
- visual processing: `5-10 FPS`
- fusion snapshot refresh: cached with short TTL to avoid hammering local APIs
- emergency gesture: sub-second after hold condition is satisfied
- event publish: best effort, non-blocking to the main loop

## Resource Limits
- grayscale processing
- bounded queues
- forensic ring buffer size derived from configured FPS and retention seconds
- degraded mode if camera, bus, or telemetry become unavailable

## Known Limits
- TA-Lib requires OHLC telemetry. Without it, candlestick output is explicit but conservative.
- Geometry detection is heuristic and should be treated as evidence, not execution logic.
- Forensics depends on local frame capture continuity; if the camera is offline, metadata is still recorded.
