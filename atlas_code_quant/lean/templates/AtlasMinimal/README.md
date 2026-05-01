# Plantilla mínima LEAN (QuantConnect)

Este directorio es un **marcador de posición** para un proyecto generado con
[Lean CLI](https://github.com/QuantConnect/lean-cli) (`lean create-project`).

Guía rápida en el repo: `scripts/bootstrap_lean_cli.ps1` en la raíz del monorepo (solo imprime pasos y variables).

Pasos recomendados:

1. Instalar el CLI de QuantConnect y Docker o runtime LEAN según la documentación oficial.
2. Crear un proyecto en `ATLAS_LEAN_PROJECT_ROOT` y un algoritmo cuyo nombre coincida con `ATLAS_LEAN_ALGORITHM`.
3. Ejecutar un backtest local y comprobar que aparece `backtests/**/statistics.json` bajo la raíz del proyecto.
4. Opcional: `ATLAS_LEAN_CLI_PASS_INTENT=true` solo si el wrapper externo entiende `--intent-json` (no forma parte del CLI estándar de QuantConnect).

El motor interno GBM (`atlas_code_quant.backtest.internal_gbm_simulator`) sigue siendo el **fallback offline** cuando LEAN no está instalado o falla con degradación activada.
