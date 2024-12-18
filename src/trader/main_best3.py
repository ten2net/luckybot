import argparse
import os
import csv
from typing import Tuple, Union
import akshare as ak
import concurrent.futures
from dotenv import load_dotenv
import schedule
import time
import pandas as pd
import pandas_ta as ta
from threading import local, Lock

from collector.akshare_data_collector import AkshareDataCollector
from core.constants import Constants
from core.topic import FavorSignalTopic, TradeSignalTopic
from filter.filter_chain import FilterChain
from filter.fund.fund_filter import NameFilter, SymbolFilter, TotalCapitalFilter
from notification.wecom import WeCom, WeComNotification
from pool.pool import ATPStockPool, AmountStockPool, FavorStockPool, HotSymbolStockPool, TurnoverStockPool
from radar.everyday_targets import EverydayTargetStockRadar
from strategy.fib import FibonacciRetracement, FibonacciTradingSignal4, VWAPCalculator
from trader.base import OrderMessage
from trader.trader_management import SimTraderManagement
from user.user_management import UserManagement
from termcolor import colored
from pubsub import pub
from datetime import datetime, timedelta

def compute_market_sentiment_index() -> float:
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

def get_stock_market(stock_code):
    """
    根据A股股票代码判断其所属的市场。

    参数:
    stock_code -- 股票代码，字符串类型

    返回:
    市场名称，若无法识别则返回 "Unknown"
    """
    if stock_code.startswith(('60', '68')):
        return "SH"
    elif stock_code.startswith(('0', '3')):
        return "SZ"
    elif stock_code.startswith(('4', '8')):
        return "BJ"
    else:
        return "Unknown"
      
def get_stock_data(stock_code):
    stock_df = None
    dt=datetime.now()
    today=dt.strftime("%Y-%m-%d")        
    stock_df =  ak.stock_zh_a_hist_min_em(symbol=stock_code, period="1",  start_date=f"{today} 09:30:00", adjust="")
    #stock_df =  ak.stock_zh_a_hist_pre_min_em(symbol=stock_code, start_time="09:25:00")
    # print(stock_df.columns)
    stock_df.rename(
        columns={"时间":"time","开盘":"open","最高":"high","最低":"low","收盘":"close","成交量":"volume","成交额":"amount"},inplace=True)    

    stock_df['high'] = stock_df['high'].astype(float)
    stock_df['low'] = stock_df['low'].astype(float)
    stock_df['close'] = stock_df['close'].astype(float)
    stock_df['volume'] = stock_df['volume'].astype(int)
    stock_df=stock_df[stock_df['volume']> 0]
    return stock_df  
# 定义处理单只股票的函数
def process_stock_data(stock):
    global ganzhou_index
    stock_symbol=stock["code"]
    stock_name=stock["name"]
    try:
          stock_tick_df = get_stock_data(stock_symbol)
          volume = stock_tick_df['volume'].sum()        
          amount = stock_tick_df['amount'].sum()            
          latest_data = stock_tick_df.iloc[-1].to_dict()
          dt = datetime.strptime(stock_tick_df.iloc[-1]['time'], "%Y-%m-%d %H:%M:%S") 
          strategyName = "择时优选"            
          date_object = datetime.strptime(latest_data['time'], "%Y-%m-%d %H:%M:%S")
          pct =round(100* (latest_data['close'] - stock["close_yesterday"]) /stock["close_yesterday"] ,2)
          turnover =round(100 * volume / (stock["circulating_capital"] / stock["close"]) ,2)
          traderMessage = {**stock,
                          "strategyName":strategyName, 
                          "resistances":"  ".join(map(str,[])),
                          "supports":"  ".join(map(str,[])),
                          "close":latest_data['close'],
                          "high":0,
                          "low":0,
                          "pct":pct,
                          "turnover":round(100 * turnover,2),
                          "amount":amount,
                          "price":latest_data['close'],
                          "index":ganzhou_index,
                          "date":date_object.strftime("%Y-%m-%d"),
                          "time":date_object.strftime("%H:%M")                            
                          }
          if latest_data['close'] != stock['lower_limit']:
              traderMessage["price"] =  latest_data['close']  
              if not args.dev: 
                  favor_message={
                  "group_name": '买信号',
                  "symbols": [stock_symbol],
                  "daily":True
                  }                        
                  pub.sendMessage(str(FavorSignalTopic.UPDATE_FAVOR),message=favor_message)                         
                  pub.sendMessage(str(TradeSignalTopic.BUY), message=traderMessage)
          elif turnover > 0.15:
              traderMessage["price"] = latest_data['close']  # 跌停价买入
              if not args.dev: 
                  favor_message={
                  "group_name": '买信号',
                  "symbols": [stock_symbol],
                  "daily":True
                  }                        
                  pub.sendMessage(str(FavorSignalTopic.UPDATE_FAVOR),message=favor_message)                         
                  pub.sendMessage(str(TradeSignalTopic.SELL),message=traderMessage) 
          else:
              pass
    except Exception as e:
        print(f"处理股票 {stock_symbol} 时发生错误: {e}")
# 使用ThreadPoolExecutor来并行处理股票池中的股票
def job():           
    global ganzhou_index
    # 计算情绪指数
    try:
        ganzhou_index = compute_market_sentiment_index()
    except Exception as e:
        print(f'计算情绪指数时发生错误: {e}')

    with concurrent.futures.ThreadPoolExecutor() as executor:
        # 1 准备市场行情快照，获取股票代码、名称、总市值、PE、PB等指标，用于筛选股票
      try:
          akshareDataCollector = AkshareDataCollector()
          market_spot_df = akshareDataCollector.get_stock_zh_a_spot_em()
          # market_spot_df = akshareDataCollector.get_fund_etf_spot_em()
        #   market_spot_df = market_spot_df[Constants.SPOT_EM_COLUMNS]
      except Exception as e:
          print(f'Akshare接口调用异常{e}')
          return
      # 2、准备主股票池和其他附加股票池        
      # favorStockPool =FavorStockPool(["自选股","仓","大笔买全榜","热股强全榜"])
      favorStockPool =FavorStockPool(["每日情全榜","封神榜全榜","牛"])
      # favorStockPool =FavorStockPool(["ETF"])
      stockPools = [favorStockPool]
      symbols = []
      for stockPool in stockPools:
          symbols += stockPool.get_symbols()
      symbols_spot_df = market_spot_df[market_spot_df['code'].isin(symbols)]
      print("股票池：",len(symbols_spot_df))
      # symbols = symbols_spot_df[['code',"name","open","high","low","pct","amount","volume_ratio","turnover","5_minute_change"]].values.tolist()
      symbols = symbols_spot_df.to_dict('records')
      if args.mt:
          executor.map(process_stock_data, symbols)
      else:
        for symbol in symbols[:20]:
            # if symbol['code'] == '000628' or symbol['code'] == '300086':  #高新发展
            process_stock_data(symbol)
        
def is_trading_time(now):
    """
    检查当前时间是否在交易时段内。
    交易时段为每个交易日的9:26到15:00。
    """
    start_hour_9 = 9
    start_minute = 26
    
    end_hour_11 = 11
    start_hour_13 = 13
    end_hour_15 = 15
    # 构建交易开始和结束的时间
    start_1 = datetime(now.year, now.month, now.day, start_hour_9, start_minute)
    end_1 = datetime(now.year, now.month, now.day, end_hour_11, 30)
    
    start_2 = datetime(now.year, now.month, now.day, start_hour_13, 0)
    end_2 = datetime(now.year, now.month, now.day, end_hour_15, 0)
    
    return now.weekday() < 5 and ((start_1 <= now <= end_1) or (start_2 <= now <= end_2))
# 设置定时任务
def timed_job(): 
    now = datetime.now()
    if is_trading_time(now):
        job()
    else:
        print(f"当前时间 {now.hour}:{now.minute} 不在交易时段")    
    
def main():
    global args
    parser = argparse.ArgumentParser(description="处理命令行参数")
    parser.add_argument('--dev', action='store_true',
                        help='Set dev mode to true')
    parser.add_argument('--daily', action='store_true',
                        help='Set k line is daily true') 
    parser.add_argument('--mt', action='store_true',
                        help='Set multithread true')       
    args = parser.parse_args()  
    os.environ['DEV_MODE'] = str(args.dev)
    os.environ['DAILY_MODE'] = str(args.daily)
    os.environ['MULTITHREAD'] = str(args.mt)       
    # load_dotenv()  # Load environment variables from .env file
    os.environ.pop("EM_APPKEY")
    os.environ.pop("EM_HEADER")
    os.environ.pop("USER_CONFIG_LIST")
    os.environ.pop("ACCOUNT_STRATEGT_MAPPING")
    os.environ.pop("WECOM_GROUP_BOT_KEYS")   
   
    load_dotenv() 
    um = UserManagement()  # 启动用户的自选股更新信号侦听
    um.startWatch()    
    stm = SimTraderManagement()  # 启动模拟账户的模拟交易信号侦听
    stm.startWatch()      
    weComNotification = WeComNotification()  # 启动企业微信通知侦听
    weComNotification.startWatch()      
    if args.dev:
        job()
    else:
        # 定时任务，每60秒执行一次，会丢数据，改成45秒执行一次
        schedule.every(45).seconds.do(timed_job)
        # 运行定时任务
        while True:
            schedule.run_pending()
            time.sleep(1)
if __name__ == "__main__":   
    main()        
    
