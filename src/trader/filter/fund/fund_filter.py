from ..base import StockFilter
from typing import List
import pandas as pd


class SymbolFilter(StockFilter):
    def __init__(self, not_allowed_prefixes: List[str] = ['4', '8', '9' , '68']):
        """
        两市剔除科创板股、北交所股
        """
        self.prefixes = not_allowed_prefixes

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:
        for prefix in self.prefixes:
            df = df[~df['code'].astype(str).str.startswith(prefix)]
        return df


class NameFilter(StockFilter):
    def __init__(self, not_allowed_prefixes: List[str] = ['ST', '*ST', 'N', 'C']):
        """
        两市剔除ST股、新股
        """
        self.prefixes = not_allowed_prefixes

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:
        for prefix in self.prefixes:
            df = df[~df['name'].astype(str).str.startswith(prefix)]
        return df


class PEFilter(StockFilter):
    def __init__(self, threshold: float = 100.0):
        """
        两市剔除市盈率大于threshold的股票
        """
        self.threshold = threshold

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:
        return df[df['pe'] < self.threshold]


class TotalCapitalFilter(StockFilter):
    def __init__(self, min_threshold: int = 30, max_threshold: int = 800):
        """
        两市剔除总市值大于min_threshold亿小于max_threshold亿的股票
        """
        self.min_threshold = min_threshold * 10000 * 10000
        self.max_threshold = max_threshold * 10000 * 10000

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:
        return df[(df['total_capital'] > self.min_threshold) & (df['total_capital'] < self.max_threshold)]


class CirculatingCapitalFilter(StockFilter):
    def __init__(self, min_threshold: int = 30, max_threshold: int = 600):
        """
        两市剔除流通市值大于min_threshold亿小于max_threshold亿的股票
        """
        self.min_threshold = min_threshold * 10000 * 10000
        self.max_threshold = max_threshold * 10000 * 10000

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:
        return df[(df['circulating_capital'] > self.min_threshold) & (df['circulating_capital'] < self.max_threshold)]
