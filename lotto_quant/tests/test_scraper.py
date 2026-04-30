"""Unit tests for the NCEL scraper (parser-only — no network calls)."""

from __future__ import annotations

import pytest

from lotto_quant.data.ncel_scraper import NCELScraper
from lotto_quant.data.scratchsmarter_scraper import ScratchOddsScraper


SAMPLE_HTML = """
<html><body>
  <div class="game" data-game-id="555">
    <h2 class="game-name">Mega Cash 7s</h2>
    <span class="ticket-price">$5</span>
    <table>
      <tr class="prize-tier"><td>$1,000,000</td><td>4</td><td>3</td></tr>
      <tr class="prize-tier"><td>$10,000</td><td>20</td><td>10</td></tr>
      <tr class="prize-tier"><td>$100</td><td>10000</td><td>2000</td></tr>
    </table>
  </div>
  <div class="game" data-game-id="556">
    <h3 class="game-name">$2 Lucky 7</h3>
    <span class="ticket-price">$2</span>
    <table>
      <tr class="prize-tier"><td>$50,000</td><td>5</td><td>4</td></tr>
      <tr class="prize-tier"><td>$10</td><td>1000</td><td>700</td></tr>
    </table>
  </div>
</body></html>
"""


def test_parse_prizes_table_extracts_two_games():
    scraper = NCELScraper()
    games = scraper.parse_prizes_table(SAMPLE_HTML)
    assert len(games) == 2
    names = sorted([g.name for g in games])
    assert "Mega Cash 7s" in names


def test_validate_data_keeps_valid():
    scraper = NCELScraper()
    games = scraper.parse_prizes_table(SAMPLE_HTML)
    valid = scraper.validate_data(games)
    assert len(valid) == 2


def test_extract_jackpot_amount_billion():
    scraper = NCELScraper()
    html = "<html><body>Jackpot tonight: $1.2 Billion</body></html>"
    amount = scraper._extract_jackpot_amount(html)
    assert amount == pytest.approx(1_200_000_000)


def test_extract_jackpot_amount_million():
    scraper = NCELScraper()
    html = "<html><body>Estimated jackpot $650 Million</body></html>"
    amount = scraper._extract_jackpot_amount(html)
    assert amount == pytest.approx(650_000_000)


def test_reconcile_returns_report():
    scraper = NCELScraper()
    games = scraper.parse_prizes_table(SAMPLE_HTML)
    recon = ScratchOddsScraper.reconcile(games, games)
    assert recon["n_primary"] == recon["n_secondary"] == recon["n_common"] == 2
    assert recon["discrepancies"] == []
