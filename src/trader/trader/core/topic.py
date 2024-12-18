from enum import Enum

class FavorSignalTopic(Enum):
    UPDATE_FAVOR = 'update.favor.signal'
    def __str__(self):
        # 只返回枚举成员的名称，不包括类名
        return self.value 
      
class TradeSignalTopic(Enum):
    BATCH_BUY = 'batch.buy.signal'
    BUY = 'buy.signal'
    SELL = 'sell.signal'
    SELL_ALL = 'sell.all.signal'
    SELL_HALF = 'sell.half.signal'
    SELL_QUARTER = 'sell.quarter.signal'
    CANCEL_ORDER = 'cancel.order.signal'
    def __str__(self):
        # 只返回枚举成员的名称，不包括类名
        return self.value         