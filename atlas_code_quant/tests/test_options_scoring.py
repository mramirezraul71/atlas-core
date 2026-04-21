import pytest
from unittest.mock import patch, MagicMock
import pandas as pd
from datetime import datetime, timedelta

from atlas_code_quant.options_scoring import (
    VolData, GexData, OiData, PriceRegime, GlobalRegime,
    SCORING_CONFIG,
    calculate_vol_score, calculate_gamma