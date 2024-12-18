from ..base import StockFilter
from typing import List
import pandas as pd


class HighVolumeFilter(StockFilter):
    def __init__(self, threshold: float = 1.5):
        """
        两市剔除上个交易日显著放量的股票
        Args:
            times_threshold 上个交易日的成交量与5日均量的倍数        
        """
        self.times_threshold = threshold

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:      
        return df[(df['volume_yestday'] / df['volume_3']) < self.times_threshold]
      
class VolumeFilter(StockFilter):
    def __init__(self, threshold: int = 5000):
        """
        上个交易日成交量大于threshold手的股票
        """
        self.threshold = threshold * 100

    def filter(self, df: pd.DataFrame) -> pd.DataFrame:
        return df[df['volume_yestday'] >= self.threshold]