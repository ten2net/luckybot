from typing import Dict
import json

from luckybot.toolkit.trader.sim.trader import Trader
from ..strategy.base import MarketValueStrategy


class SellOnConsecutiveDeclines(MarketValueStrategy):
    def __init__(self, threshold):
        self.threshold = threshold

    def execute(self, market_data: Dict, account_data: Dict, trader: Trader):
        # 检查是否连续两周市值下降，并执行卖空操作
        print("market_data:",json.dumps(market_data, ensure_ascii=False))
        print("account_data:",json.dumps(account_data, ensure_ascii=False))