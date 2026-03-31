"""Genetic Strategy Evolver — Auto-Strategy Discovery for ATLAS.

Uses genetic programming to generate, mutate, crossover, and select
trading strategy variants.  Each "genome" is a set of indicator
parameters + entry/exit rules that can be backtested via VectorBT.

Pipeline:
    1. Generate initial population of strategy genomes
    2. Backtest each genome on historical data (VectorBT vectorized)
    3. Score: IC > 0.05 AND Sharpe > 1.0 pass the filter
    4. Select top performers, crossover, mutate → next generation
    5. Survivors get deployed to paper trading for validation

Usage:
    evolver = StrategyEvolver()
    results = evolver.evolve(symbol="SPY", generations=20, population=50)
    # results.best_genomes → top strategies ready for paper deploy
"""
from __future__ import annotations

import copy
import hashlib
import json
import logging
import random
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger("atlas.strategy_evolver")

# ── Genome Definition ──────────────────────────────────────────────

# Available indicator genes and their valid ranges
GENE_POOL = {
    "rsi_period":       {"min": 5,   "max": 30,   "type": "int"},
    "rsi_oversold":     {"min": 15,  "max": 40,   "type": "int"},
    "rsi_overbought":   {"min": 60,  "max": 85,   "type": "int"},
    "ma_fast":          {"min": 5,   "max": 30,   "type": "int"},
    "ma_slow":          {"min": 20,  "max": 100,  "type": "int"},
    "atr_period":       {"min": 7,   "max": 21,   "type": "int"},
    "atr_sl_mult":      {"min": 1.0, "max": 4.0,  "type": "float"},
    "atr_tp_mult":      {"min": 1.5, "max": 6.0,  "type": "float"},
    "bb_period":        {"min": 10,  "max": 30,   "type": "int"},
    "bb_std":           {"min": 1.5, "max": 3.0,  "type": "float"},
    "macd_fast":        {"min": 8,   "max": 16,   "type": "int"},
    "macd_slow":        {"min": 20,  "max": 30,   "type": "int"},
    "macd_signal":      {"min": 7,   "max": 12,   "type": "int"},
    "volume_sma":       {"min": 10,  "max": 30,   "type": "int"},
    "volume_threshold": {"min": 1.2, "max": 3.0,  "type": "float"},
}

# Entry rule templates (combinable)
ENTRY_RULES = [
    "rsi_oversold",          # RSI < oversold threshold
    "ma_cross_up",           # fast MA crosses above slow MA
    "bb_lower_touch",        # price touches lower Bollinger Band
    "macd_cross_up",         # MACD crosses above signal
    "volume_spike",          # volume > threshold × SMA
    "rsi_divergence",        # price lower low + RSI higher low
]

EXIT_RULES = [
    "rsi_overbought",        # RSI > overbought threshold
    "ma_cross_down",         # fast MA crosses below slow MA
    "bb_upper_touch",        # price touches upper BB
    "atr_trailing_stop",     # trailing stop at ATR multiple
    "fixed_rr",              # fixed risk:reward ratio
    "time_decay",            # exit after N bars
]


@dataclass
class StrategyGenome:
    """A single strategy variant — the 'DNA' of a trading strategy."""

    genome_id: str = ""
    generation: int = 0
    genes: dict[str, float] = field(default_factory=dict)
    entry_rules: list[str] = field(default_factory=list)
    exit_rules: list[str] = field(default_factory=list)

    # Fitness metrics (filled after backtest)
    sharpe: float = 0.0
    ic: float = 0.0
    win_rate: float = 0.0
    profit_factor: float = 0.0
    total_trades: int = 0
    max_drawdown: float = 0.0
    expectancy_r: float = 0.0

    # Lineage
    parents: list[str] = field(default_factory=list)
    mutation_log: list[str] = field(default_factory=list)

    def __post_init__(self):
        if not self.genome_id:
            self.genome_id = self._make_id()

    def _make_id(self) -> str:
        raw = json.dumps({"g": self.genes, "e": self.entry_rules, "x": self.exit_rules}, sort_keys=True)
        return "GEN-" + hashlib.md5(raw.encode()).hexdigest()[:10]

    @property
    def fitness(self) -> float:
        """Composite fitness score for selection."""
        if self.total_trades < 5:
            return -999.0
        # Weighted: Sharpe (40%) + IC (30%) + PF (20%) + WR (10%)
        score = (self.sharpe * 0.40 +
                 self.ic * 100 * 0.30 +  # IC ~0.05 → 5 points
                 min(self.profit_factor, 5.0) * 0.20 +
                 (self.win_rate - 50) * 0.10)
        # Penalty for excessive drawdown
        if self.max_drawdown < -25:
            score *= 0.5
        return round(score, 4)

    @property
    def passes_filter(self) -> bool:
        """Check if genome meets minimum deployment criteria."""
        return (self.ic > 0.05 and
                self.sharpe > 1.0 and
                self.total_trades >= 10 and
                self.profit_factor > 1.1)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["fitness"] = self.fitness
        d["passes_filter"] = self.passes_filter
        return d


# ── Genetic Operations ─────────────────────────────────────────────

def random_genome(generation: int = 0) -> StrategyGenome:
    """Create a random strategy genome."""
    genes = {}
    for name, spec in GENE_POOL.items():
        if spec["type"] == "int":
            genes[name] = random.randint(spec["min"], spec["max"])
        else:
            genes[name] = round(random.uniform(spec["min"], spec["max"]), 2)

    # Ensure ma_fast < ma_slow
    if genes["ma_fast"] >= genes["ma_slow"]:
        genes["ma_fast"], genes["ma_slow"] = genes["ma_slow"], genes["ma_fast"]
    if genes["macd_fast"] >= genes["macd_slow"]:
        genes["macd_fast"], genes["macd_slow"] = genes["macd_slow"], genes["macd_fast"]

    # Random subset of entry/exit rules (2-3 each)
    entry = random.sample(ENTRY_RULES, k=random.randint(1, 3))
    exit_ = random.sample(EXIT_RULES, k=random.randint(1, 3))

    return StrategyGenome(generation=generation, genes=genes,
                          entry_rules=entry, exit_rules=exit_)


def mutate(genome: StrategyGenome, rate: float = 0.2) -> StrategyGenome:
    """Mutate a genome by randomly perturbing some genes."""
    child = copy.deepcopy(genome)
    child.parents = [genome.genome_id]
    child.mutation_log = []

    for name, spec in GENE_POOL.items():
        if random.random() > rate:
            continue
        old = child.genes.get(name, spec["min"])
        delta_range = (spec["max"] - spec["min"]) * 0.15
        if spec["type"] == "int":
            delta = random.randint(-int(delta_range), int(delta_range))
            child.genes[name] = max(spec["min"], min(spec["max"], int(old) + delta))
        else:
            delta = random.uniform(-delta_range, delta_range)
            child.genes[name] = round(max(spec["min"], min(spec["max"], old + delta)), 2)
        child.mutation_log.append(f"{name}: {old} -> {child.genes[name]}")

    # Occasionally swap a rule
    if random.random() < 0.15 and child.entry_rules:
        idx = random.randrange(len(child.entry_rules))
        new_rule = random.choice(ENTRY_RULES)
        if new_rule not in child.entry_rules:
            child.mutation_log.append(f"entry[{idx}]: {child.entry_rules[idx]} -> {new_rule}")
            child.entry_rules[idx] = new_rule

    if random.random() < 0.15 and child.exit_rules:
        idx = random.randrange(len(child.exit_rules))
        new_rule = random.choice(EXIT_RULES)
        if new_rule not in child.exit_rules:
            child.mutation_log.append(f"exit[{idx}]: {child.exit_rules[idx]} -> {new_rule}")
            child.exit_rules[idx] = new_rule

    # Fix constraints
    if child.genes.get("ma_fast", 0) >= child.genes.get("ma_slow", 100):
        child.genes["ma_fast"], child.genes["ma_slow"] = child.genes["ma_slow"], child.genes["ma_fast"]

    child.genome_id = child._make_id()
    return child


def crossover(a: StrategyGenome, b: StrategyGenome) -> StrategyGenome:
    """Uniform crossover between two genomes."""
    child_genes = {}
    for name in GENE_POOL:
        child_genes[name] = a.genes.get(name) if random.random() < 0.5 else b.genes.get(name)

    # Mix rules
    all_entry = list(set(a.entry_rules + b.entry_rules))
    all_exit = list(set(a.exit_rules + b.exit_rules))
    entry = random.sample(all_entry, k=min(len(all_entry), random.randint(1, 3)))
    exit_ = random.sample(all_exit, k=min(len(all_exit), random.randint(1, 3)))

    child = StrategyGenome(
        genes=child_genes,
        entry_rules=entry,
        exit_rules=exit_,
        parents=[a.genome_id, b.genome_id],
    )
    child.genome_id = child._make_id()
    return child


# ── Backtester (VectorBT-powered) ─────────────────────────────────

def _backtest_genome(genome: StrategyGenome, ohlcv: Any) -> StrategyGenome:
    """Backtest a single genome using vectorized numpy operations.

    Uses VectorBT for performance when available, falls back to
    pure numpy vectorized simulation.

    Args:
        genome: Strategy genome to test.
        ohlcv: DataFrame/dict with columns: open, high, low, close, volume.
               Can be pandas, polars, or dict of numpy arrays.
    """
    try:
        import polars as pl

        # Convert to numpy arrays for speed
        if isinstance(ohlcv, pl.DataFrame):
            close = ohlcv["close"].to_numpy().astype(np.float64)
            high = ohlcv["high"].to_numpy().astype(np.float64)
            low = ohlcv["low"].to_numpy().astype(np.float64)
            volume = ohlcv["volume"].to_numpy().astype(np.float64) if "volume" in ohlcv.columns else np.ones(len(close))
        elif isinstance(ohlcv, dict):
            close = np.asarray(ohlcv["close"], dtype=np.float64)
            high = np.asarray(ohlcv["high"], dtype=np.float64)
            low = np.asarray(ohlcv["low"], dtype=np.float64)
            volume = np.asarray(ohlcv.get("volume", np.ones(len(close))), dtype=np.float64)
        else:
            # pandas DataFrame
            close = ohlcv["close"].values.astype(np.float64)
            high = ohlcv["high"].values.astype(np.float64)
            low = ohlcv["low"].values.astype(np.float64)
            volume = ohlcv["volume"].values.astype(np.float64) if "volume" in ohlcv.columns else np.ones(len(close))
    except Exception as e:
        logger.warning("Backtest data conversion failed: %s", e)
        return genome

    n = len(close)
    if n < 100:
        return genome

    g = genome.genes

    # ── Compute indicators ─────────────────────────────────────────
    # RSI
    rsi_period = int(g.get("rsi_period", 14))
    deltas = np.diff(close)
    gains = np.where(deltas > 0, deltas, 0.0)
    losses = np.where(deltas < 0, -deltas, 0.0)

    avg_gain = np.zeros(n)
    avg_loss = np.zeros(n)
    if rsi_period < n:
        avg_gain[rsi_period] = np.mean(gains[:rsi_period])
        avg_loss[rsi_period] = np.mean(losses[:rsi_period])
        for i in range(rsi_period + 1, n):
            avg_gain[i] = (avg_gain[i-1] * (rsi_period - 1) + gains[i-1]) / rsi_period
            avg_loss[i] = (avg_loss[i-1] * (rsi_period - 1) + losses[i-1]) / rsi_period

    rs = np.where(avg_loss > 0, avg_gain / avg_loss, 100.0)
    rsi = 100 - (100 / (1 + rs))

    # Moving averages
    ma_fast_p = int(g.get("ma_fast", 10))
    ma_slow_p = int(g.get("ma_slow", 50))
    ma_fast = np.convolve(close, np.ones(ma_fast_p)/ma_fast_p, mode='same')
    ma_slow = np.convolve(close, np.ones(ma_slow_p)/ma_slow_p, mode='same')

    # ATR
    atr_p = int(g.get("atr_period", 14))
    tr = np.maximum(high[1:] - low[1:],
                    np.maximum(np.abs(high[1:] - close[:-1]),
                               np.abs(low[1:] - close[:-1])))
    tr = np.insert(tr, 0, high[0] - low[0])
    atr = np.convolve(tr, np.ones(atr_p)/atr_p, mode='same')

    # Volume SMA
    vol_sma_p = int(g.get("volume_sma", 20))
    vol_sma = np.convolve(volume, np.ones(vol_sma_p)/vol_sma_p, mode='same')

    # ── Generate entry/exit signals ────────────────────────────────
    entries = np.zeros(n, dtype=bool)
    exits = np.zeros(n, dtype=bool)

    # Entry conditions (AND logic within the genome's selected rules)
    entry_mask = np.ones(n, dtype=bool)
    for rule in genome.entry_rules:
        if rule == "rsi_oversold":
            entry_mask &= rsi < g.get("rsi_oversold", 30)
        elif rule == "ma_cross_up":
            entry_mask &= (ma_fast > ma_slow) & (np.roll(ma_fast, 1) <= np.roll(ma_slow, 1))
        elif rule == "volume_spike":
            entry_mask &= volume > (g.get("volume_threshold", 1.5) * vol_sma)
        elif rule == "macd_cross_up":
            macd_f = int(g.get("macd_fast", 12))
            macd_s = int(g.get("macd_slow", 26))
            ema_f = np.convolve(close, np.ones(macd_f)/macd_f, mode='same')
            ema_s = np.convolve(close, np.ones(macd_s)/macd_s, mode='same')
            macd_line = ema_f - ema_s
            entry_mask &= (macd_line > 0) & (np.roll(macd_line, 1) <= 0)
        elif rule == "bb_lower_touch":
            bb_p = int(g.get("bb_period", 20))
            bb_std = g.get("bb_std", 2.0)
            bb_sma = np.convolve(close, np.ones(bb_p)/bb_p, mode='same')
            bb_roll_std = np.array([np.std(close[max(0,i-bb_p):i+1]) for i in range(n)])
            bb_lower = bb_sma - bb_std * bb_roll_std
            entry_mask &= close <= bb_lower

    entries = entry_mask
    # Skip warmup period
    warmup = max(ma_slow_p, rsi_period, atr_p) + 5
    entries[:warmup] = False

    # Exit conditions (OR logic — any exit rule triggers)
    for rule in genome.exit_rules:
        if rule == "rsi_overbought":
            exits |= rsi > g.get("rsi_overbought", 70)
        elif rule == "ma_cross_down":
            exits |= (ma_fast < ma_slow) & (np.roll(ma_fast, 1) >= np.roll(ma_slow, 1))
        elif rule == "atr_trailing_stop":
            # Simplified: exit if price drops more than atr_sl_mult * ATR from recent high
            sl_mult = g.get("atr_sl_mult", 2.0)
            running_max = np.maximum.accumulate(close)
            exits |= close < (running_max - sl_mult * atr)

    exits[:warmup] = False

    # ── Simulate trades ────────────────────────────────────────────
    trades_pnl = []
    in_trade = False
    entry_price = 0.0
    entry_idx = 0

    for i in range(warmup, n):
        if not in_trade and entries[i]:
            in_trade = True
            entry_price = close[i]
            entry_idx = i
        elif in_trade and (exits[i] or i == n - 1):
            in_trade = False
            exit_price = close[i]
            pnl_pct = (exit_price - entry_price) / entry_price * 100
            risk = atr[entry_idx] * g.get("atr_sl_mult", 2.0)
            r_mult = (exit_price - entry_price) / risk if risk > 0 else 0
            trades_pnl.append({"pnl_pct": pnl_pct, "r": r_mult, "bars": i - entry_idx})

    # ── Compute fitness metrics ────────────────────────────────────
    genome.total_trades = len(trades_pnl)
    if not trades_pnl:
        return genome

    pnls = [t["pnl_pct"] for t in trades_pnl]
    rs = [t["r"] for t in trades_pnl]
    wins = [p for p in pnls if p > 0]
    losses = [p for p in pnls if p <= 0]

    genome.win_rate = round(len(wins) / len(pnls) * 100, 2) if pnls else 0
    genome.expectancy_r = round(np.mean(rs), 4) if rs else 0

    gross_win = sum(wins) if wins else 0
    gross_loss = abs(sum(losses)) if losses else 0
    genome.profit_factor = round(gross_win / gross_loss, 3) if gross_loss > 0 else (999 if gross_win > 0 else 0)

    mean_pnl = np.mean(pnls)
    std_pnl = np.std(pnls)
    genome.sharpe = round(mean_pnl / std_pnl * np.sqrt(252) if std_pnl > 0 else 0, 3)

    # Max drawdown
    cum = np.cumsum(pnls)
    peak = np.maximum.accumulate(cum)
    dd = cum - peak
    genome.max_drawdown = round(float(np.min(dd)), 2) if len(dd) else 0

    # IC approximation: correlation between entry signal strength (RSI distance from 50)
    # and realized R-multiple. Simple but meaningful
    if len(trades_pnl) >= 10:
        entry_indices = []
        in_trade = False
        for i in range(warmup, n):
            if not in_trade and entries[i]:
                in_trade = True
                entry_indices.append(i)
            elif in_trade and (exits[i] or i == n - 1):
                in_trade = False
        signal_strength = [abs(rsi[idx] - 50) / 50 for idx in entry_indices[:len(trades_pnl)]]
        r_outcomes = [t["r"] for t in trades_pnl[:len(signal_strength)]]
        if len(signal_strength) == len(r_outcomes) and len(signal_strength) >= 5:
            try:
                from scipy.stats import spearmanr
                ic, _ = spearmanr(signal_strength, r_outcomes)
                genome.ic = round(float(ic) if not np.isnan(ic) else 0, 4)
            except ImportError:
                # Fallback: simple correlation
                ss = np.array(signal_strength)
                ro = np.array(r_outcomes)
                if ss.std() > 0 and ro.std() > 0:
                    genome.ic = round(float(np.corrcoef(ss, ro)[0, 1]), 4)

    return genome


# ── Evolution Engine ───────────────────────────────────────────────

@dataclass
class EvolutionResult:
    """Result of an evolution run."""
    symbol: str
    generations: int
    population_size: int
    total_backtests: int
    duration_sec: float
    best_genomes: list[dict]
    generation_stats: list[dict]
    passing_genomes: list[dict]  # IC > 0.05 AND Sharpe > 1


class StrategyEvolver:
    """Genetic programming engine for automatic strategy discovery."""

    def __init__(self, *, results_dir: Path | None = None):
        self._results_dir = results_dir or (
            Path(__file__).resolve().parent.parent / "data" / "learning" / "evolution"
        )
        self._results_dir.mkdir(parents=True, exist_ok=True)

    def evolve(
        self,
        ohlcv: Any,
        symbol: str = "SPY",
        generations: int = 20,
        population_size: int = 50,
        elite_pct: float = 0.2,
        mutation_rate: float = 0.25,
        crossover_rate: float = 0.5,
    ) -> EvolutionResult:
        """Run the full genetic evolution loop.

        Args:
            ohlcv: OHLCV data (pandas/polars DataFrame or dict of arrays).
            symbol: Symbol being evolved for.
            generations: Number of generations to run.
            population_size: Genomes per generation.
            elite_pct: Top % to keep unchanged.
            mutation_rate: Probability of mutating each gene.
            crossover_rate: Probability of crossover vs mutation.
        """
        t0 = time.perf_counter()
        logger.info("Evolution started: symbol=%s, gen=%d, pop=%d", symbol, generations, population_size)

        # Initialize population
        population = [random_genome(generation=0) for _ in range(population_size)]
        gen_stats = []
        total_backtests = 0

        for gen in range(generations):
            # Backtest all genomes
            for genome in population:
                genome.generation = gen
                _backtest_genome(genome, ohlcv)
                total_backtests += 1

            # Sort by fitness
            population.sort(key=lambda g: g.fitness, reverse=True)

            # Stats
            fitnesses = [g.fitness for g in population]
            passing = [g for g in population if g.passes_filter]
            stat = {
                "generation": gen,
                "best_fitness": round(fitnesses[0], 4),
                "avg_fitness": round(np.mean(fitnesses), 4),
                "median_fitness": round(np.median(fitnesses), 4),
                "passing": len(passing),
                "best_sharpe": population[0].sharpe,
                "best_ic": population[0].ic,
                "best_trades": population[0].total_trades,
            }
            gen_stats.append(stat)

            if gen % 5 == 0 or gen == generations - 1:
                logger.info(
                    "[Gen %d/%d] best=%.2f avg=%.2f passing=%d sharpe=%.2f ic=%.4f",
                    gen, generations, stat["best_fitness"], stat["avg_fitness"],
                    stat["passing"], stat["best_sharpe"], stat["best_ic"],
                )

            # Selection + breeding
            elite_n = max(2, int(population_size * elite_pct))
            elite = population[:elite_n]
            next_gen = list(elite)  # Elites survive unchanged

            while len(next_gen) < population_size:
                if random.random() < crossover_rate and len(elite) >= 2:
                    # Crossover two random elites
                    a, b = random.sample(elite, 2)
                    child = crossover(a, b)
                    if random.random() < mutation_rate:
                        child = mutate(child, rate=mutation_rate)
                else:
                    # Mutate a random elite
                    parent = random.choice(elite)
                    child = mutate(parent, rate=mutation_rate)
                next_gen.append(child)

            population = next_gen[:population_size]

        # Final sort
        population.sort(key=lambda g: g.fitness, reverse=True)
        duration = time.perf_counter() - t0

        # Save results
        passing = [g for g in population if g.passes_filter]
        result = EvolutionResult(
            symbol=symbol,
            generations=generations,
            population_size=population_size,
            total_backtests=total_backtests,
            duration_sec=round(duration, 2),
            best_genomes=[g.to_dict() for g in population[:10]],
            generation_stats=gen_stats,
            passing_genomes=[g.to_dict() for g in passing],
        )

        self._save_result(result)
        logger.info(
            "Evolution complete: %d backtests in %.1fs, %d strategies pass filter",
            total_backtests, duration, len(passing),
        )
        return result

    def _save_result(self, result: EvolutionResult) -> Path:
        ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        path = self._results_dir / f"evolution_{result.symbol}_{ts}.json"
        path.write_text(json.dumps({
            "symbol": result.symbol,
            "generations": result.generations,
            "population_size": result.population_size,
            "total_backtests": result.total_backtests,
            "duration_sec": result.duration_sec,
            "best_genomes": result.best_genomes[:5],
            "passing_count": len(result.passing_genomes),
            "passing_genomes": result.passing_genomes[:10],
            "generation_stats": result.generation_stats,
        }, indent=2, default=str), encoding="utf-8")
        logger.info("Evolution results saved to %s", path)
        return path
