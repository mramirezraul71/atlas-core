# Atlas Index — Biblioteca de Conocimiento ATLAS-Quant
> Actualizado: 2026-03-29 | Fuentes: 22 | Métodos scanner cubiertos: 6

---

## CAT1 — Teoría Base

| ID | Título | Año | Confianza | Temporalidad |
|----|--------|-----|-----------|--------------|
| `fama_1965_rw` | Random Walks in Stock-Market Prices | 1965 | high | all |
| `ohara_1995_mmt` | Market Microstructure Theory | 1995 | high | intraday, short |
| `lo_mackinlay_1988_vr` | Stock Market Prices Do Not Follow Random Walks | 1988 | high | short, swing |

**Qué resuelven**: ¿Es predecible el precio? ¿Cuál es la hipótesis nula que cualquier señal debe superar?

---

## CAT2 — Fenómenos de Precio

| ID | Título | Año | Confianza | Temporalidad |
|----|--------|-----|-----------|--------------|
| `jegadeesh_titman_1993` | Returns to Buying Winners and Selling Losers | 1993 | high | swing, medium |
| `moskowitz_ooi_pedersen_2012` | Time Series Momentum | 2012 | high | swing, medium |
| `debondt_thaler_1985` | Does the Stock Market Overreact? | 1985 | high | long |
| `jegadeesh_1990_reversal` | Evidence of Predictable Behavior of Security Returns | 1990 | medium | intraday, short |
| `asness_moskowitz_pedersen_2013` | Value and Momentum Everywhere | 2013 | high | swing, medium |
| `faber_2007_tactical` | A Quantitative Approach to Tactical Asset Allocation | 2007 | high | medium |
| `lo_mackinlay_1990_overreactions` | When Are Contrarian Profits Due to Overreaction? | 1990 | medium | short, swing |

**Qué resuelven**: ¿Cómo se comportan los retornos en distintos horizontes? ¿Dónde está el edge?

---

## CAT3 — Patrones Visuales

| ID | Título | Año | Confianza | Temporalidad |
|----|--------|-----|-----------|--------------|
| `lo_mamaysky_wang_2000` | Foundations of Technical Analysis | 2000 | medium | short, swing |
| `prechter_frost_1978_elliott` | Elliott Wave Principle | 1978 | low | all |
| `poser_2003_elliott_effectiveness` | The Effectiveness of the Elliott Waves Theory | 2003 | medium | all |

**Qué resuelven**: ¿Tienen los patrones chartistas evidencia empírica? ¿Cuáles son más robustos?

> ⚠️ Elliott Waves: evidencia baja — usar solo como contexto visual, NO como trigger de entrada.

---

## CAT4 — Modelos Explicativos

| ID | Título | Año | Confianza | Temporalidad |
|----|--------|-----|-----------|--------------|
| `daniel_hirshleifer_subrahmanyam_1998` | Investor Psychology and Under/Overreactions | 1998 | medium | swing, medium |
| `barberis_thaler_2003` | A Survey of Behavioral Finance | 2003 | high | all |
| `grinold_kahn_2000_apm` | Active Portfolio Management | 2000 | high | all |
| `perold_1988_is` | The Implementation Shortfall | 1988 | high | intraday, short |
| `black_scholes_1973` | The Pricing of Options | 1973 | high | all |
| `natenberg_1994_options_volatility` | Option Volatility and Pricing | 1994 | high | short, swing |

**Qué resuelven**: ¿Por qué existen las anomalías? ¿Cómo medir calidad de señal? ¿Cómo valuar opciones?

---

## CAT5 — Metodología

| ID | Título | Año | Confianza | Temporalidad |
|----|--------|-----|-----------|--------------|
| `tsay_2010_fts` | Analysis of Financial Time Series | 2010 | high | short, swing, medium |
| `lopez_de_prado_2018_afml` | Advances in Financial Machine Learning | 2018 | high | all |
| `van_tharp_1999_riskmanagement` | Trade Your Way to Financial Freedom | 1999 | high | all |
| `kaminski_lo_2014_stoploss` | When Do Stop-Loss Rules Stop Losses? | 2014 | high | short, swing |
| `almgren_chriss_2001_execution` | Optimal Execution of Portfolio Transactions | 2001 | high | intraday |
| `brinson_hood_beebower_1986` | Determinants of Portfolio Performance | 1986 | high | medium, long |

**Qué resuelven**: ¿Cómo modelar, validar y ejecutar? ¿Cómo medir si el sistema funciona?

---

## Mapa por Método Scanner ATLAS

| Método | Fuentes que lo respaldan | Evidence Score |
|--------|--------------------------|----------------|
| `trend_ema_stack` | moskowitz_ooi_pedersen_2012, jegadeesh_titman_1993, lo_mackinlay_1988_vr, faber_2007_tactical, grinold_kahn_2000_apm, tsay_2010_fts | 0.88 |
| `breakout_donchian` | moskowitz_ooi_pedersen_2012, lo_mackinlay_1988_vr, kaminski_lo_2014_stoploss, tsay_2010_fts, grinold_kahn_2000_apm | 0.82 |
| `rsi_pullback_trend` | jegadeesh_1990_reversal, daniel_hirshleifer_subrahmanyam_1998, van_tharp_1999_riskmanagement, grinold_kahn_2000_apm | 0.66 |
| `relative_strength_overlay` | jegadeesh_titman_1993, asness_moskowitz_pedersen_2013, lo_mackinlay_1990_overreactions | 0.84 |
| `order_flow_proxy` | ohara_1995_mmt | 0.79 |
| `ml_directional` | lopez_de_prado_2018_afml, grinold_kahn_2000_apm | 0.58 |

---

## Mapa por Temporalidad

| Temporalidad | Fuentes más relevantes | Fenómenos dominantes |
|--------------|----------------------|---------------------|
| Intraday | ohara_1995_mmt, perold_1988_is, almgren_chriss_2001_execution | Microestructura, order flow, bid-ask |
| Short (1-5d) | jegadeesh_1990_reversal, lo_mackinlay_1988_vr, kaminski_lo_2014_stoploss | Reversión débil, bid-ask bounce |
| Swing (5-60d) | jegadeesh_titman_1993, moskowitz_ooi_pedersen_2012, tsay_2010_fts | Momentum, continuación de tendencia |
| Medium (60-252d) | asness_moskowitz_pedersen_2013, faber_2007_tactical, brinson_hood_beebower_1986 | Momentum cross-seccional, ciclos |
| Long (>252d) | debondt_thaler_1985, barberis_thaler_2003 | Reversión a la media, fundamentales |

---

## Criterios para ir a Live (Lopez de Prado + Grinold-Kahn)

| Criterio | Umbral | Fuente |
|---------|--------|--------|
| IC del scanner | > 0.05 (meaningful) / > 0.10 (fuerte) | Grinold & Kahn (2000) |
| t-stat IC | ≥ 2.0 | Grinold & Kahn (2000) |
| N trades OOS mínimo | ≥ 30 para IC, ≥ 100 para Sharpe | Lopez de Prado (2018) |
| Deflated Sharpe Ratio | > 0 | Lopez de Prado (2018) |
| MTRL | ≥ 3 meses OOS | Lopez de Prado (2018) |
| Implementation Shortfall | < 0.5% por operación | Perold (1988) |
| R-múltiple promedio | > 0.5R | Van Tharp (1999) |

---

## Líneas de investigación futuras (atlas_notes)

- **Volatility targeting**: Kelly dinámico con GARCH (Tsay 2010 + Kelly Engine)
- **Sector rotation**: evidencia de momentum sectorial (Fama-French 3-factor)
- **Options edge**: IV mean reversion cuantificada (Natenberg, CBOE research)
- **Crypto momentum**: literatura emergente (Liu, Tsyvinski 2021)
- **Intraday momentum**: Gao, Han, Li, Zhou (2018) — momentum del primer y último minuto
- **Earnings momentum**: PEAD (post-earnings announcement drift) — Bernard & Thomas (1989)
