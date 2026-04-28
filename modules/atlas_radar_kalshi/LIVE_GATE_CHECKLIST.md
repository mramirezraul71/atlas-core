# Radar Live Promotion Checklist

Checklist operativo para promoción `paper -> live` con evidencia verificable:

1. `critical_findings_closed`: no hallazgos críticos abiertos (`RADAR_AUDIT_CRITICAL_OPEN=0`).
2. `drawdown_within_limits`: drawdown en paper dentro de límite (`RADAR_PROMO_MAX_DD_CENTS`).
3. `loss_streak_within_limits`: racha de pérdidas <= límite (`RADAR_PROMO_MAX_LOSS_STREAK`).
4. `auto_rollback_tested`: rollback automático validado en ventana post-aplicación.
5. `critical_tests_enabled`: suite crítica ejecutada y estable.
6. `kill_switch_available`: kill-switch y recuperación de safe-mode disponibles en dashboard.

API de evidencia:

- `GET /api/radar/live-gate/checklist`
- `GET /api/radar/shadow`
- `GET /api/radar/metrics`
