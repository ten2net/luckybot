from datetime import datetime
import math
from typing import List
import os

import pandas as pd
from collector.akshare_data_collector import AkshareDataCollector
from trader.base import OrderMessage, TradeSignalHandler
from user.user_management import UserManagement
class SimTraderManagement(TradeSignalHandler):
    def __init__(self):
      um = UserManagement() 
      users = um.get_users()
      self.trading_accounts=[]
      akshareDataCollector = AkshareDataCollector()
      self.market_spot_df_all = akshareDataCollector.get_stock_zh_a_spot_em()

      for user in users:
         self.trading_accounts += user.get_accounts()
         
    def on_buy_signal(self, message:dict, **kwargs):
        orderMessage = OrderMessage(strategyName=message["strategyName"], symbol=message["code"], price=message["price"], pct=message["pct"] ,index=message["index"])
        self.commit_buy_orders(strategyName=message["strategyName"], orderMessages = [orderMessage], position_ratio=message["index"])
    def on_batch_buy_signal(self, message:List[OrderMessage], **kwargs):
        strategyName=message[0].strategyName
        index=message[0].index
        self.commit_buy_orders(strategyName=strategyName, orderMessages = message, position_ratio=index)
    
    def on_sell_signal(self,  message:dict, **kwargs):
      orderMessage = OrderMessage(strategyName=message["strategyName"], symbol=message["code"], price=message["price"], pct=message["pct"] ,index=message["index"])
      self.commit_sell_order(orderMessage, 1) 
    def on_sell_half_signal(self, message:OrderMessage, **kwargs):
      self.commit_sell_order(message, 1/2) 
    def on_sell_quarter_signal(self, message:OrderMessage, **kwargs):
        self.commit_sell_order(message, 1/4)         
    def on_sell_all_signal(self, message:OrderMessage, **kwargs):
        self.commit_sell_order(message, -1)          
    def on_cancel_order_signal(self,  message:OrderMessage, **kwargs):
      print(f"----cancel_order_signal {message.__dict__}!")    
         
    def commit_buy_orders(self,strategyName:str,orderMessages:List[OrderMessage], position_ratio:float):
       if os.getenv('DEV_MODE', 'False') == 'True':
        return 
       for account in self.trading_accounts: 
         if account.strategyName == strategyName:         
           stock_prices={orderMessage.symbol:orderMessage.price for orderMessage in orderMessages}
           buy_batch_result = account.trader.execute_buy(stock_prices=stock_prices, position_ratio=position_ratio)
           print(account.accountName, buy_batch_result)
    def commit_sell_order(self, message:OrderMessage, position_rate: float):
      if os.getenv('DEV_MODE', 'False') == 'True':
        return      
      msg ="一键清仓 " if position_rate <0 else "平仓" if position_rate == 1 else "平半" if position_rate==0.5 else "平四分之一" 
      print(f"{datetime.now()},卖出信号:{msg}, 当前情绪指数:{message.index}")
      # return 
      strategyName = message.strategyName
      index = message.index
      symbol=message.symbol 
      for account in self.trading_accounts: 
        if account.strategyName == strategyName: 
          # print(account.accountName, 50 * "-")
          position = account.trader.get_position()
          if position is not None and len(position) > 0:
              df_hold = pd.DataFrame(data=position)
              df_hold['quantity'] = df_hold['quantity'].astype(int)
              df_hold['quantity_can_use'] = df_hold['quantity_can_use'].astype(int)
              df_hold['purchase_price'] = df_hold['purchase_price'].astype(float)
              
              # 过滤掉不能卖出的股票，避免废单
              df_can_sell =df_hold[df_hold['quantity_can_use'] > 0].copy()
              df_can_sell['upper_rate'] = df_can_sell['code'].apply(lambda x: 20 if (x.startswith('3') or x.startswith('68'))  else 10)
              df_can_sell = df_can_sell.merge(self.market_spot_df_all, on="code", how="left")
              for index, row in df_can_sell.iterrows():
                  quantity_can_use = abs(row['quantity_can_use'] * position_rate)
                  quantity_can_use = quantity_can_use - (quantity_can_use % 100)  # 确保是整手
                  if row['code'] == message.symbol:
                    sell_price =max(message.price, row["lower_limit"]) 
                    account.trader.sell(message.symbol, price=sell_price, stock_num=quantity_can_use)
                  # else:
                  #   # 一键清仓时不检查行情，更低价卖出
                  #   sell_price =max(round(row['close'] * 0.985, 2), row["lower_limit"]) if position_rate<0 else max(round(row['close'] * 0.995, 2), row["lower_limit"])  # 确保尽量能出手
                  #   # 当日停牌的股票 sell_price 为 nan，需要过滤掉
                  #   # sell_signal =not math.isnan(sell_price) if position_rate<0 else row['close'] < row['open'] or abs(row['pct'] - row['upper_rate']) >= 0.1 or math.isnan(sell_price)
                  #   sell_signal =True if (not math.isnan(sell_price)) and (row['close'] < row['open'] or abs(row['pct'] - row['upper_rate']) >= 0.1) else False
                  #   if not sell_signal: continue                    
                  #   account.trader.sell(row['code'], price=sell_price, stock_num=quantity_can_use)
        