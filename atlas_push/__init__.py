"""atlas_push — núcleo de Atlas Push.

Paquete que alberga el cerebro de Atlas Push. Contenido actual (Paso B):

- ``atlas_push.intents``: `IntentRouter` y `IntentResult`.

Próximos pasos (no implementados aún):

- ``atlas_push.engine``: `DecisionEngine` y pipeline de decisión (Paso D).
- ``atlas_push.strategies``: `Strategy.propose` (Paso D).
- ``atlas_push.risk``: `RiskManager.apply` (Paso D).
- ``atlas_push.state``: `MarketState` y tipos asociados (Paso D).
- ``atlas_push.outputs``: `DecisionOutput`, `LogicalOrder`, etc. (Paso D).
- ``atlas_push.ports``: Protocols hacia sistemas externos (Paso E).
- ``atlas_push.config``: carga de configuración (Paso E).

Regla de oro: ``atlas_push`` no importa símbolos del brain core mayor de
ATLAS. Los contratos viajan vía ``typing.Protocol``.
"""
