from dotenv import load_dotenv
import os
import json

from trader.sim.trader import Trader

class Account:
    trader:Trader = None
    def __init__(self, return_info: dict = {}, balance_info: dict = {}):
        self.__dict__.update(return_info)
        self.__dict__.update(balance_info)
        
        load_dotenv()
        account_strategy_config = json.loads(
            os.environ.get('ACCOUNT_STRATEGT_MAPPING'))
        self.strategyName = account_strategy_config.get(self.accountName)        
    def set_trader(self,trader: Trader):
        self.trader = trader
