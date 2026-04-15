# atlas-core
ATLAS Core — Hands of the RAULI Robot (tools, runners, memory, audit)

- **Deploy (Blue-Green + Canary):** [docs/DEPLOY.md](docs/DEPLOY.md)

## Atlas Code Quant - PDT Rule Elimination (2026-04-14)

### What Changed

SEC approved elimination of Pattern Day Trading rule on April 14, 2026.

#### Before (PDT Rule Active)
- Max 4 day trades per 5 calendar days
- $25,000 minimum account capital required
- Buying power calculated at end of day (EOD)
- Account freeze 90 days if capital drops below $25K

#### After (PDT Rule Eliminated)
- 4-trade limit removed
- $25K minimum removed
- Real-time buying power (calculated intraday)
- No account freeze (no PDT designation)

### What Atlas Did

#### Removed
- Blocking behavior from legacy `check_pdt_limit()` gate
- $25,000 minimum capital as a regulatory blocker
- PDT-driven opening blocks in Tradier execution

#### Added
- Real-time operational gates (non-regulatory):
  - Daily trade limit: 50/day (live), 100/day (paper)
  - Intraday exposure limit: max 50% of equity
  - Intraday drawdown limit: max 10% loss per day
  - Consecutive loss cooldown: 3+ losses trigger pause

### Impact for Your Bot

**Good news:**
- Unlimited day trading now possible (no 4-trade cap)
- Can operate with lower capital amounts (no $25K threshold)
- Operational checks are now risk-driven instead of PDT-driven

**Responsibility:**
- Operational gates now prevent overtrading and blowups
- Monitor drawdown and exposure continuously
- Cooldown logic reduces cascading losses

### Broker Implementation

- April-May 2026: Early adopters (possibly Tradier)
- June-Oct 2026: Most brokers
- By 2027: Universal compliance

Atlas migration is ready before full broker rollout.
