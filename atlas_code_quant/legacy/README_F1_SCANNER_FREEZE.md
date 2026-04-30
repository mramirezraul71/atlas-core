# F1 Scanner Freeze (sin cutover)

En F1 se prioriza seguridad operativa. Por eso:

- `atlas_code_quant/scanner/` **no** se movió físicamente todavía.
- No se eliminaron endpoints `/scanner/*`.
- No se tocó el flujo productivo de `api/main.py` asociado al scanner.

## Motivo

Mover el scanner completo en F1 elevaría el riesgo de rotura por el número
de imports y referencias cruzadas. El corte se difiere a F3 con shadow-mode.

## Plan de migración

1. Introducir intake Radar estable.
2. Activar flag `ATLAS_LEGACY_SCANNER_ENABLED` para controlar desuso.
3. Ejecutar shadow comparativo.
4. Recién en F3, mover/eliminar scanner y endpoints legacy.
