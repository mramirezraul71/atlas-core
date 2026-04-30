# Prompt maestro XGBoost — índice por secciones (referencia 13 Abr 2026)

Orden lógico del documento fuente (texto completo lo conservas tú en `.docx` / pegado en chat):

| Orden | Sección | Contenido resumido |
|------|---------|-------------------|
| 0 | Contexto y restricciones | Mapa ATLAS, restricciones, sin tocar archivos críticos salvo hooks acordados |
| 1 | Arquitectura del módulo | Paquete `learning/xgboost_signal/`, exports, dependencias |
| 2 | Feature engineering | Pre-trade, in-trade, targets, `OptionsFeatureBuilder` placeholder |
| 3 | Fases de datos | Fase 0/1/2, tabla `xgboost_feature_log`, umbrales |
| 4 | Integración | Settings `QUANT_XGBOOST_*`, router API, `evaluate_for_risk_guard` |
| 5 | Walk-forward | Ventanas, métricas, IC Spearman, baseline, overfitting |
| 6 | Exit advisor | `ExitAdvice`, sin `override_governance` |
| 7 | Audit report | JSON + Markdown, ALERTS |
| 8 | Opciones futuro | Extensión sin refactor mayor |
| 9 | Entregables y criterios | Checklist de aceptación |

**Implementación en repo:** `atlas_code_quant/learning/xgboost_signal/`  
**API:** `atlas_code_quant/api/routers/xgboost_router.py` (incluido en `api/main.py`)  
**Tests:** `tests/test_xgboost_signal.py`  
**Demo:** `scripts/xgboost_demo.py`
