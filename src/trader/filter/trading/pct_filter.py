from ..base import StockFilter
from typing import List
import pandas as pd


class PCTFilter(StockFilter):
    def __init__(self, min_threshold: float = 3.0, max_threshold: float = 20.5):
        """
        上个交易日涨跌幅大于min_threshold小于max_threshold的股票
        """
        self.min_threshold = min_threshold
        self.max_threshold = max_threshold

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:
        return df[(df['pct_yestday'] > self.min_threshold) & (df['pct_yestday'] < self.max_threshold)]


      
