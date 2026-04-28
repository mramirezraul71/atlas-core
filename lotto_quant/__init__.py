"""
Atlas Lotto-Quant Module
========================

Quantitative analysis engine for the North Carolina Education Lottery (NCEL).
Probabilistic arbitrage of:
  1. Scratch-off prize depletion asymmetry (stale prizes)
  2. Jackpot rollover EV (Powerball / Mega Millions)
  3. Markov Chain state modeling of game lifecycles

This module is AI-model-agnostic and integrates with Atlas's local-AI stack
(Ollama / DeepSeek / Qwen3) via `lotto_quant.llm.local_client`.
"""

__version__ = "1.0.0"
__author__ = "Atlas Project"
