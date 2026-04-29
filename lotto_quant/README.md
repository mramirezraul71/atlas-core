# Atlas Lotto-Quant Module

Quantitative arbitrage of the **North Carolina Education Lottery (NCEL)**.
Part of the [Atlas](https://github.com/mramirezraul71/atlas-core) autonomous
trading and decision-support stack.

> **TL;DR** — This module systematically detects EV-positive scratch-off and
> draw-game opportunities by combining prize-depletion asymmetry, Markov-Chain
> state modeling, and tax-adjusted Kelly sizing. All AI is **local**
> (Ollama / DeepSeek / Qwen3) — Atlas remains air-gapped.

---

## Strategy

1. **Scratch-off Stale-Prize Arbitrage**
   When > 75 % of a game's tickets are sold but > 80 % of its major prizes
   remain, the conditional odds for the remaining tickets are dramatically
   better than the headline odds.
2. **Jackpot Rollover EV (Powerball / Mega Millions)**
   When the jackpot crosses the EV-positive threshold *after* accounting for
   tax withholding and jackpot-splitting risk.
3. **Markov Chain Trajectory Modeling**
   Monte-Carlo trajectories of remaining-prize vectors predict when EV will
   turn positive on a given game.

## Quick Start

```bash
# 1. Install deps
pip install -r requirements_lotto.txt

# 2. Start a local model (Ollama)
ollama pull qwen3:14b
ollama serve  # default :11434

# 3. (Optional) Configure environment
export ATLAS_LLM_MODEL=qwen3:14b
export ATLAS_TELEGRAM_TOKEN=...
export ATLAS_TELEGRAM_CHAT_ID=...

# 4. Smoke test
python -m lotto_quant.signals.opportunity_radar --bankroll 1000 --once

# 5. Live HUD
streamlit run lotto_quant/dashboard/hud.py
```

## Architecture

```
                      ┌─────────────────────┐
                      │   NCEL.com (HTML)   │
                      └──────────┬──────────┘
                                 ▼
                ┌────────────────────────────┐
                │    NCELScraper (httpx)     │
                │  (LLM fallback if needed)  │
                └──────────┬─────────────────┘
                           ▼
        ┌───────────────────────────────────────────┐
        │         EVCalculator   ←   Markov          │
        │  (gross EV, tax-adjusted EV, anomaly)      │
        └──────────┬─────────────────┬───────────────┘
                   ▼                 ▼
            SignalGate        JackpotSimulator
                   │                 │
                   └───────┬─────────┘
                           ▼
                    KellyAllocator
                           │
                ┌──────────┴──────────┐
                ▼                     ▼
           AlertEngine            DuckDB
       (Telegram/console)
```

## Configuration

All knobs live in `lotto_quant/config.py` and can be overridden by env vars.

| Variable | Default | Description |
| --- | --- | --- |
| `ATLAS_LLM_BACKEND` | `ollama` | `ollama` or `openai_compat` |
| `ATLAS_LLM_URL` | `http://localhost:11434` | Local model server URL |
| `ATLAS_LLM_MODEL` | `qwen3:14b` | Primary model identifier |
| `ATLAS_LLM_ENABLED` | `true` | Disable to run pure deterministic mode |
| `ATLAS_NC_TAX` | `0.0525` | NC state withholding |
| `ATLAS_FED_TAX` | `0.24` | Federal withholding ≥ $5,000 |
| `ATLAS_SCRAPE_INTERVAL` | `3600` | Radar interval in seconds |
| `ATLAS_TELEGRAM_TOKEN` | `""` | Telegram bot token (optional) |
| `ATLAS_TELEGRAM_CHAT_ID` | `""` | Telegram destination chat |
| `ATLAS_LOTTO_DB` | `data/lotto_quant.duckdb` | DuckDB path |

## Local-AI Usage (Important)

Atlas Lotto-Quant uses local LLMs **only** for:
- Generating Telegram-ready prose from numeric signals.
- Falling back to model-extracted JSON when BeautifulSoup yields nothing.
- Optional risk-officer review of borderline Kelly allocations.

The math is **never** in the prompt. All numerical decisions live in
`models/` and are fully deterministic and unit-tested.

To check whether the local backend is reachable:
```bash
python -c "import asyncio; from lotto_quant.llm import LocalAIClient; \
print(asyncio.run(_check())) " <<'PY'
async def _check():
    async with LocalAIClient() as ai:
        return {'healthy': await ai.health(), 'models': await ai.list_models()}
PY
```

Or open the **Streamlit HUD** sidebar — it shows backend status live.

## Mathematical Reference

**Adjusted EV (post-tax)**
```
EV_gross    = Σ ( P(win_i) · prize_i ) − ticket_price
P(win_i)    = remaining_prizes_i / remaining_tickets
prize_net   = prize · (1 − NC_TAX − FED_TAX·[prize ≥ 5000])
EV_adj      = Σ ( P(win_i) · prize_i_net ) − ticket_price
```

**Stale-Prize Anomaly Score**
```
anomaly_score = depletion_ratio / (1 − major_prize_retention)
SIGNAL ⇔ depletion > 0.75 ∧ retention > 0.80 ⇒ score > 3.75
```

**Lottery-Adjusted Kelly**
```
f*           = (p · b − q) / b
f_atlas      = 0.25 · f*
σ            = std(prize_values) / mean(prize_values)
f_lot        = f_atlas / (1 + σ)
f_final      = min(f_lot, MAX_POSITION_PCT)
```

**Jackpot Splitting**
```
share        = jackpot / max(1, n_tickets · p_jackpot)
EV_jackpot   = share · (1 − tax) · p_jackpot − ticket_price
```

## License & Disclaimer

For research and educational use within the Atlas ecosystem. No part of this
code constitutes financial advice. Lotteries are high-variance instruments
with negative expected value in nearly all states; this module exists to find
the rare exceptions and quantify the variance.
