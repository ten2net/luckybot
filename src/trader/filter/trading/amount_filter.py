from ..base import StockFilter
from typing import List
import pandas as pd


class AmountFilter(StockFilter):
    def __init__(self, threshold: int = 10):
        """
        上个交易日成交额大于threshold亿的股票
        """
        self.threshold = threshold * 10000 * 10000

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:
        return df[df['amount_yestday'] >= self.threshold]

      
