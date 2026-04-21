from dataclasses import dataclass
from typing import Dict, Any, List, Literal
from datetime import datetime, timedelta

import pandas as pd
import yfinance as yf
from pydantic import BaseModel, Field, field_validator


@dataclass
class VolData:
    iv_rank