from abc import ABC, abstractmethod
from typing import Dict

from luckybot.toolkit.trader.sim.trader import Trader

class MarketValueStrategy(ABC):
    @abstractmethod
    def execute(self, market_data:Dict, account_data: Dict, trader: Trader):
        pass

class StrategyManager:
    def __init__(self):
        self.strategies = {}
        self.enabled_strategies = []

    def register_strategy(self, name, strategy):
        self.strategies[name] = strategy

    def enable_strategy(self, name):
        if name in self.strategies:
            self.enabled_strategies.append(name)

    def disable_strategy(self, name):
        if name in self.enabled_strategies:
            self.enabled_strategies.remove(name)

    def execute_strategies(self, market_data: Dict, account_data: Dict, trader: Trader):
        for name in self.enabled_strategies:
            self.strategies[name].execute(market_data, account_data, trader)    