from abc import ABC, abstractmethod
from typing import List
import pandas as pd
import pandas_ta as ta
import numpy as np

from pool.pool import AmountStockPool
class StockRadar(ABC):
    @abstractmethod
    def startup(self):
        pass
    def compute_market_sentiment_index(self) -> float:
        stockPool = AmountStockPool()
        df =stockPool.get_data_frame(cloumn_name="amount", k=300)       

        # 定义涨跌幅区间和权重
        bins = [-float('inf'), -0.1, -0.075, -0.05, -0.025, 0.0, 0.025, 0.05, 0.075, 0.1, float('inf')]
        weights = [-16, -8, -4, -2, -1, 1, 2, 4, 8, 16]

        df['segment'] = pd.cut(df['pct'] / 100, bins=bins, labels=weights, right=True)

        df['weighted_amount'] = df['amount'] * df['segment'].astype(float)
        # 计算实际加权因子
        actual_weighted_factor = df['weighted_amount'].sum()
     
        # 计算最大可能的加权因子
        max_weighted_factor = df['amount'].sum() * max(weights)

        # 计算强弱指数
        strength_index = round(actual_weighted_factor / max_weighted_factor,3)
        return strength_index
        