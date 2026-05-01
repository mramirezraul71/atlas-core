"""Tests for learning.strategy_evolver — Genetic Programming strategy discovery."""
import pytest

from learning.strategy_evolver import (
    StrategyGenome,
    random_genome,
    mutate,
    crossover,
    GENE_POOL,
    ENTRY_RULES,
    EXIT_RULES,
)


def test_random_genome_valid():
    g = random_genome()
    assert isinstance(g, StrategyGenome)
    # entry_rules/exit_rules are lists — each element must be from the pool
    for rule in g.entry_rules:
        assert rule in ENTRY_RULES
    for rule in g.exit_rules:
        assert rule in EXIT_RULES
    # All genes within pool bounds
    for key, val in g.genes.items():
        pool = GENE_POOL[key]
        assert pool["min"] <= val <= pool["max"], f"{key}={val} out of [{pool['min']}, {pool['max']}]"


def test_mutate_preserves_structure():
    g = random_genome()
    m = mutate(g, rate=1.0)  # 100% mutation rate
    assert isinstance(m, StrategyGenome)
    for rule in m.entry_rules:
        assert rule in ENTRY_RULES
    for rule in m.exit_rules:
        assert rule in EXIT_RULES
    for key, val in m.genes.items():
        pool = GENE_POOL[key]
        assert pool["min"] <= val <= pool["max"]


def test_mutate_zero_rate_preserves_genes():
    g = random_genome()
    m = mutate(g, rate=0.0)
    # rate=0.0 means no gene mutations, but rule swaps (15% chance) are independent
    assert m.genes == g.genes


def test_crossover_produces_valid():
    a = random_genome()
    b = random_genome()
    child = crossover(a, b)
    assert isinstance(child, StrategyGenome)
    # Each gene should come from either parent
    for key in GENE_POOL:
        assert child.genes[key] in (a.genes[key], b.genes[key])


def test_genome_fitness_default():
    g = random_genome()
    # Unbacktested genome has sentinel fitness
    assert g.sharpe == 0.0
    assert g.ic == 0.0
    assert g.profit_factor == 0.0
    assert g.win_rate == 0.0
    assert g.total_trades == 0


def test_multiple_random_genomes_differ():
    genomes = [random_genome() for _ in range(10)]
    gene_sets = [tuple(sorted(g.genes.items())) for g in genomes]
    # With randomness, very unlikely all 10 are identical
    assert len(set(gene_sets)) > 1


def test_crossover_deterministic_with_identical_parents():
    a = random_genome()
    child = crossover(a, a)
    assert child.genes == a.genes
