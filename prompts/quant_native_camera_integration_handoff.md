Implement native camera support in `atlas_code_quant` without changing the existing trading gates or execution semantics.

Context:
- PUSH already exposes a safe external bridge:
  - `POST /api/trading/quant/vision-cycle`
  - `GET /api/trading/quant/vision-bridge/status`
  - `POST /api/primitives/nexus/look-at-screen`
- Camera alignment/calibration already exists in:
  - `modules/nexus_core/screen_gaze.py`
  - `modules/nexus_core/primitives.py`
- Quant currently supports:
  - `vision_mode = off | manual | desktop_capture | insta360_pending`
  - `SensorVisionService.capture_context_snapshot()`
  - `OperationCenter.evaluate_candidate()` storing `vision_snapshot`

Goal:
- Replace `insta360_pending` with a real native provider that can either:
  1. Call PUSH bridge endpoints, or
  2. Reuse the same calibration/capture concepts directly if you intentionally want tighter coupling.

Implementation constraints:
- Preserve current operation-cycle behavior and paper-first safety gates.
- Do not weaken `operator_present`, `screen_integrity_ok`, or emergency-stop semantics.
- Keep `manual` and `desktop_capture` working exactly as they do today.
- Additive change only: do not break existing API payloads.

Requested changes:
1. In `atlas_code_quant/api/schemas.py`
   - Extend `vision_mode` to include a real provider such as `atlas_push_bridge`.

2. In `atlas_code_quant/operations/sensor_vision.py`
   - Add support for provider `atlas_push_bridge`.
   - Read bridge settings from env vars:
     - `ATLAS_PUSH_BASE_URL` default `http://127.0.0.1:8791`
     - `ATLAS_PUSH_TIMEOUT_SEC` default `20`
     - Optional `ATLAS_SCREEN_GAZE_CALIB_PATH`
   - Implement `capture_context_snapshot()` for this provider by calling PUSH bridge endpoints.
   - Persist returned capture path / metadata just like existing providers.
   - Report `provider_ready = true` when bridge health says it is ready.

3. In `atlas_code_quant/operations/operation_center.py`
   - Keep current `vision_snapshot` shape stable.
   - Continue using `self.vision.capture_context_snapshot(...)`.
   - No behavior changes outside the new provider.

4. In `atlas_code_quant/api/main.py`
   - No broad refactor.
   - Only ensure config/status surfaces can expose the new `vision_mode`.

5. Add tests
   - Provider readiness for `atlas_push_bridge`
   - Snapshot capture through mocked PUSH bridge responses
   - Operation cycle still emits `vision_snapshot`

Bridge contract:
- `GET /api/trading/quant/vision-bridge/status`
  returns readiness, calibration path, prompt path and endpoint inventory.
- `POST /api/trading/quant/vision-cycle`
  accepts:
  {
    "order": {...},
    "action": "evaluate|preview|submit",
    "include_vision": true,
    "capture_target": "nexus:camera|nexus:screen|screen",
    "screen_target": {"x": 1920, "y": 1080, "zoom": 1.2},
    "calib_path": "optional path",
    "source": "camera",
    "quant_capture_context": false,
    "timeout_sec": 20
  }

Safety note:
- The bridge is intended to avoid editing `atlas_code_quant` while another agent is active.
- Once the repo slice is free, implement the native provider cleanly inside quant and then decide whether to keep or retire the PUSH bridge.
