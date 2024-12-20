from abc import ABC, abstractmethod
from typing import List
from pubsub import pub

from core.topic import TradeSignalTopic

def subscribe_topic(topic_name):
    def decorator(callback):
        pub.subscribe(topic_name, callback)
        return callback
    return decorator
  
class OrderMessage:
    def __init__(self,strategyName:str, symbol:str, price:float, pct:float,index:float):
        self.strategyName = strategyName
        self.symbol = symbol
        self.price = price
        self.pct = pct
        self.index=index
        
class TradeSignalHandler(ABC):
    @abstractmethod
    def on_buy_signal(self, message:OrderMessage ):
        pass
    @abstractmethod
    def on_batch_buy_signal(self, message:List[OrderMessage] ):
        pass

    @abstractmethod
    def on_sell_signal(self,  message:OrderMessage ):
        pass
    @abstractmethod
    def on_sell_all_signal(self, message:OrderMessage, **kwargs):
        pass
    @abstractmethod
    def on_sell_half_signal(self, message:OrderMessage, **kwargs):
        pass
    @abstractmethod
    def on_sell_quarter_signal(self, message:OrderMessage, **kwargs):
        pass

    @abstractmethod
    def on_cancel_order_signal(self,  message:OrderMessage ):
        pass     
    def startWatch(self):
      pub.subscribe(self.on_buy_signal, str(TradeSignalTopic.BUY))
      pub.subscribe(self.on_sell_signal, str(TradeSignalTopic.SELL))
      pub.subscribe(self.on_sell_half_signal, str(TradeSignalTopic.SELL_HALF))
      pub.subscribe(self.on_sell_quarter_signal, str(TradeSignalTopic.SELL_QUARTER))
      pub.subscribe(self.on_sell_all_signal, str(TradeSignalTopic.SELL_ALL))
      pub.subscribe(self.on_cancel_order_signal, str(TradeSignalTopic.CANCEL_ORDER))
      pub.subscribe(self.on_batch_buy_signal, str(TradeSignalTopic.BATCH_BUY))

