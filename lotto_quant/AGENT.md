# Atlas Lotto-Quant — Agent Context

> Renamed from `CLAUDE.md` to be model-agnostic. Atlas now runs **fully local
> AI** (Ollama / DeepSeek / Qwen3 / Llama) for any LLM-assisted task.

## Module Purpose
Quantitative analysis of NC Education Lottery (NCEL) for Expected-Value
arbitrage. This is **not** gambling — it is systematic exploitation of
prize-depletion asymmetry.

## Key Concepts
- **Stale-Prize Anomaly** — `>75 %` tickets sold but `>80 %` major prizes
  remaining → real odds improved over stated odds.
- **Adjusted EV** — Gross EV minus NC state tax (5.25 %) and federal
  withholding (24 % for prizes ≥ $5,000).
- **Markov Chain** — Models prize-state transitions as tickets are sold.
- **Quarter-Kelly** — `f_atlas = 0.25 × full_kelly`, plus a variance penalty
  `1 / (1 + σ_prizes / μ_prizes)` to handle lottery's extreme variance.

## Local AI
The LLM is used **only** for:
1. Narrating EV signals into Telegram-ready prose.
2. Fallback HTML-to-JSON extraction when the BeautifulSoup parser yields zero
   rows (NCEL markup occasionally changes).
3. Optional risk-officer review of borderline Kelly allocations.

The math is **never** delegated to the model. All numerical computation lives
in `models/` and is fully deterministic.

Configure the local backend via env vars:
```
ATLAS_LLM_BACKEND=ollama          # 'ollama' or 'openai_compat'
ATLAS_LLM_URL=http://localhost:11434
ATLAS_LLM_MODEL=qwen3:14b
```
Recommended model rotation:
- **Primary**: `qwen3:14b`  (good tool-following, JSON discipline)
- **Fallback**: `deepseek-r1:14b`  (deeper reasoning when needed)
- **Backup**:   `llama3.1:8b`  (always-available, fast)

## Module Structure
```
lotto_quant/
├── config.py
├── llm/
│   ├── local_client.py    # Ollama / OpenAI-compat client
│   └── prompts.py
├── models/
│   ├── ev_calculator.py
│   ├── markov_scratchoff.py
│   ├── jackpot_simulator.py
│   └── kelly_allocator.py
├── data/
│   ├── ncel_scraper.py
│   ├── scratchsmarter_scraper.py
│   ├── schemas.py
│   └── database.py
├── signals/
│   ├── opportunity_radar.py
│   ├── signal_gate.py
│   └── alert_engine.py
├── vision/ticket_ocr.py
├── dashboard/hud.py
├── watchdog/lotto_watchdog.py
└── tests/
```

## Integration Points
- `atlas-core/radar/` — Lotto-Quant radar registers as a signal source.
- `atlas-core/risk/` — Kelly recommendations flow through the global risk manager.
- `atlas-core/vision/` — Insta360 capture is shared with the main vision module.
- `atlas-core/alerts/` — Telegram bot uses the shared alert bus.

## Running

### Single cycle (smoke test)
```bash
python -m lotto_quant.signals.opportunity_radar --bankroll 1000 --once
```

### Continuous radar
```bash
python -m lotto_quant.signals.opportunity_radar --bankroll 1000 --interval 3600
```

### Dashboard
```bash
streamlit run lotto_quant/dashboard/hud.py
```

### Tests
```bash
pytest lotto_quant/tests -v
```

## Guidance for Agents Working on This Module
1. **Never** call an external LLM API. All AI must go through
   `lotto_quant.llm.local_client.LocalAIClient`.
2. **Never** put numerical decisions inside a prompt. The LLM is for narration
   and triage only.
3. Tests must run **without** network access and **without** Ollama running —
   the LLM must be optional at every call site.
4. Keep dependencies pinned in `requirements_lotto.txt`.
