from ..base import StockFilter
from typing import List
import pandas as pd


class TurnoverFilter(StockFilter):
    def __init__(self, threshold: float = 5):
        """
        上个交易日换手率大于threshold的股票
        """
        self.threshold = threshold

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:
        return df[df['turnover_yestday'] >= self.threshold]

      
