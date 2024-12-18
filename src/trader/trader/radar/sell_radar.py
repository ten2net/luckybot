from datetime import datetime
import math
from typing import List
from core.constants import Constants
from collector.akshare_data_collector import AkshareDataCollector
from core.topic import FavorSignalTopic, TradeSignalTopic
from filter.filter_chain import FilterChain
from filter.fund.fund_filter import NameFilter, SymbolFilter, TotalCapitalFilter
from filter.trading.amount_filter import AmountFilter
from notification.wecom import WeComNotification
from radar.base import StockRadar
from pool.pool import HotSymbolStockPool
import os
from termcolor import colored
from pubsub import pub
from trader.base import OrderMessage


import pandas as pd
import pandas_ta as ta

class BollingerOBVStrategy:
    def __init__(self, window_bb=14, window_obv=14, window_signal=14):
        self.window_bb = window_bb
        self.window_obv = window_obv
        self.window_signal = window_signal
        self.obv_history = []  # 用于存储OBV历史数据
        self.data = pd.DataFrame(columns=['close', 'volume', 'bb_upper', 'bb_middle', 'bb_lower', 'obv'])

    def update_data(self, new_close, new_volume):
        # 将新数据添加到DataFrame
        new_data = pd.DataFrame({'close': [new_close], 'volume': [new_volume]})
        self.data = pd.concat([self.data, new_data]) if not self.data.empty else new_data

        # 计算OBV并更新历史
        obv = ta.obv(new_close, new_volume, self.obv_history)
        self.obv_history.append(obv)
        self.data['obv'] = self.data['close'] * 0  # 初始化OBV列
        self.data.loc[len(self.data) - 1, 'obv'] = obv

        # 计算布林带
        if len(self.data) >= self.window_bb:
            self.data['bb_upper'], self.data['bb_middle'], self.data['bb_lower'] = ta.bbands(
                self.data['close'],
                length=self.window_bb
            )

        # 计算移动平均
        if len(self.data) >= self.window_signal:
            self.data['close_ma'] = self.data['close'].rolling(window=self.window_signal).mean()
            self.data['obv_ma'] = self.data['obv'].rolling(window=self.window_signal).mean()

        # 生成信号
        self.generate_signals()

    def generate_signals(self):
        # 清除旧信号
        self.data['signal'] = 0

        # 检查背离并生成信号
        if len(self.data) >= 2:
            bearish_divergence = (self.data['close'].iloc[-1] > self.data['close'].iloc[-2]) and (
                self.data['obv'].iloc[-1] < self.data['obv_ma'].iloc[-2]
            )
            bullish_divergence = (self.data['close'].iloc[-1] < self.data['close'].iloc[-2]) and (
                self.data['obv'].iloc[-1] > self.data['obv_ma'].iloc[-2]
            )

            # 卖出信号：看跌背离且最新价接近布林带上轨
            if bearish_divergence and self.data['close'].iloc[-1] > self.data['bb_upper'].iloc[-1]:
                self.data.at[len(self.data) - 1, 'signal'] = -1

            # 买入信号：看涨背离且最新价接近布林带下轨
            elif bullish_divergence and self.data['close'].iloc[-1] < self.data['bb_lower'].iloc[-1]:
                self.data.at[len(self.data) - 1, 'signal'] = 1

    def get_last_signal(self):
        return self.data.iloc[-1]['signal']

# 使用示例
strategy = BollingerOBVStrategy()
# 假设实时数据流中新的价格和成交量
new_close = 150.5
new_volume = 2000000
strategy.update_data(new_close, new_volume)
last_signal = strategy.get_last_signal()
print(f"Last signal: {last_signal}")

class SellStockRadar(StockRadar):
    def __init__(self, name: str = "热股强势", topN: int = 22):
        self.name = name
        self.topN = min(topN, 22)

    def startup(self):
        # 1 准备市场行情快照，获取股票代码、名称、总市值、PE、PB等指标，用于筛选股票
        try:
            akshareDataCollector = AkshareDataCollector()
            market_spot_df = akshareDataCollector.get_stock_zh_a_spot_em()
            market_spot_df_all = market_spot_df
            market_spot_df = market_spot_df[Constants.SPOT_EM_COLUMNS_BASE]
        except Exception as e:
            print(f'Akshare接口调用异常{e}')
            return
        # 2、准备主股票池和其他附加股票池
        mainStockPool = HotSymbolStockPool()
        stockPools = [mainStockPool]
        symbols = []
        for stockPool in stockPools:
            symbols += stockPool.get_symbols(k=5)
        # 3、先对symbols进行基本面过滤,以便减少后续计算量
        symbols_spot_df = market_spot_df[market_spot_df['code'].isin(symbols)]
        fand_filter_list = [SymbolFilter(),
                            NameFilter(),
                            TotalCapitalFilter(
                                min_threshold=30, max_threshold=1200),  # 总市值过滤
                            ]
        fand_filter_chain = FilterChain(fand_filter_list)
        symbols_spot_df = fand_filter_chain.apply(symbols_spot_df)
        symbols = symbols_spot_df['code'].tolist()
        # 4、获取股票数据，并附加其他指标
        # df = mainStockPool.get_data_with_indictores(symbols,withCDL=False)
        df = mainStockPool.get_data(symbols)
        if df.shape[0] == 0:
            print(colored("未到集合竞价时间！", 'red'))
            return
        df = df.merge(market_spot_df, on="code", how="left")
        # 5、附加其他指标
        # 6、筛选股票，实现单独的过滤器，添加到过滤器链中即可
        filters = [
            AmountFilter(threshold=1.5),  # 昨日成交额过滤器，过滤掉成交额小于2亿的股票
            # HighVolumeFilter(threshold=2), # 昨日成交量过滤器，过滤掉成交量大于5日均量1.3倍的股票
        ]
        filter_chain = FilterChain(filters)
        df = filter_chain.apply(df)
        
        df['upper_rate'] = df['code'].apply(lambda x: 20 if (
            x.startswith('3') or x.startswith('68')) else 10)

        df['space_limit'] = (
            df['upper_limit_y'] - df["close"]) / df['upper_limit_y']        

        if df.shape[0] == 0:
            print(colored("无满足条件的股票！", 'yellow'))
        else:
            # 计算情绪指数
            ganzhou_index = self.compute_market_sentiment_index()
            # 7、评分
            # df['score'] = PriceScorer().score(df) + VolumeScorer().score(df) + PEScorer().score(df) +
            #               SentimentScorer().score(df)

            # 8、排序
            df = df.head(round(1.5 * self.topN))  # 自选股不受长度限制
            df['is_hot_industry'] = True # 该策略全部是热门行业

            df.sort_values(by=['is_hot_industry', 'pct'], ascending=[False, False], inplace=True)
            df = df.reset_index(drop=True)
            print(colored(f"""{self.name}发现了 {df.shape[0]} 个目标：{df['name'].tolist()}""", "green"))
            # 9、自选股
            try:
                results = df['code'].tolist()
                results = results[::-1]  # 确保新加自选的在上面
                favor_message={
                  "group_name": self.name,
                  "symbols": results
                }
                pub.sendMessage(str(FavorSignalTopic.UPDATE_FAVOR),message=favor_message)
            except Exception as e:
                print(f'东方财富接口调用异常:{e}')
            #10、交易信号生成，主程序中启动的模拟盘交易管理器SimTraderManagement负责侦听交易信号，实施交易
            try:            
                now = datetime.now()
                if ganzhou_index < 0.03:   # 情绪太差,只卖不买
                  if (now.hour==9 and now.minute <=45) or (now.hour==14 and now.minute <=10):   # 发出一键清仓信号
                    pub.sendMessage(str(TradeSignalTopic.SELL_ALL),message=OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index))
                  else:
                    pub.sendMessage(str(TradeSignalTopic.SELL),message=OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)) 
                elif 0.05 <= ganzhou_index < 0.12: # 情绪一般,适度买卖
                  if (now.hour==9 and now.minute <=45) :   # 开盘15分钟只买不卖
                    if now.minute >10:
                      # 卖出信号生成，不涨停就先卖出1/4                   
                      sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
                      pub.sendMessage(str(TradeSignalTopic.SELL_QUARTER),message=sell_message)                     
                    # 买入信号生成
                    df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 5)]  # 过滤掉当日上涨空间已经不多的股票                    
                    df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
                    price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
                    df_buy = df_buy.head(5)  # 只买最强的5个股票
                    orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
                        row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
                    pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
                  elif (now.hour == 9 and now.minute > 45) or (9 < now.hour <= 14) :  # 下午两点之前只买不卖
                    # 买入信号生成
                    df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 3)]  # 过滤掉上涨空间还有三分之一的股票                         
                    df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
                    df_buy = df_buy[df_buy['pct'] < 5]  # 只买红票
                    price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
                    df_buy = df_buy.head(5)  # 只买最强的5个股票
                    orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
                        row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
                    pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
                  else:   # 下午两点之后只卖不买
                    # 卖出信号生成
                    sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
                    pub.sendMessage(str(TradeSignalTopic.SELL),message=sell_message)
                elif 0.12 <= ganzhou_index < 0.22: # 情绪良好,多买少卖
                  if (now.hour==9 and now.minute <=45) :   # 开盘15分钟只买不卖
                    if now.minute >10:
                      # 卖出信号生成，不涨停就先卖出1/2                   
                      sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
                      pub.sendMessage(str(TradeSignalTopic.SELL_HALF),message=sell_message)                     
                    # 买入信号生成
                    df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 5)]  # 过滤掉当日上涨空间已经不多的股票                    
                    df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
                    price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
                    df_buy = df_buy.head(10)  # 只买最强的10个股票
                    orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
                        row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
                    pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
                  elif (now.hour == 9 and now.minute > 45) or (9 < now.hour <= 14) :  # 下午两点之前只买不卖
                    # 买入信号生成
                    df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 3)]  # 过滤掉上涨空间还有三分之一的股票                         
                    df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
                    df_buy = df_buy[df_buy['pct'] < 5]  # 只买红票
                    price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
                    df_buy = df_buy.head(10)  # 只买最强的10个股票
                    orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
                        row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
                    pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
                  else:   # 下午两点之后只卖不买
                    # 卖出信号生成                    
                    sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
                    pub.sendMessage(str(TradeSignalTopic.SELL),message=sell_message)
                elif 0.22 <= ganzhou_index < 0.33: # 情绪很好,只买不卖
                  if (now.hour==9 and now.minute <=45) :   # 开盘15分钟
                    if now.minute >10:
                      # 卖出信号生成，不涨停就先卖出                   
                      sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
                      pub.sendMessage(str(TradeSignalTopic.SELL),message=sell_message)                     
                    # 买入信号生成
                    df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 5)]  # 过滤掉当日上涨空间已经不多的股票                    
                    df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
                    price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
                    df_buy = df_buy.head(10)  # 只买最强的10个股票
                    orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
                        row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
                    pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
                    # 卖出信号生成，不涨停就卖                   
                    sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
                    pub.sendMessage(str(TradeSignalTopic.SELL_HALF),message=sell_message)                    
                  elif (now.hour == 9 and now.minute > 45) or (9 < now.hour <= 14) :  # 下午两点之前只买不卖
                    # 买入信号生成
                    df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 3)]  # 过滤掉上涨空间还有三分之一的股票                         
                    df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
                    df_buy = df_buy[df_buy['pct'] < 5]  # 只买红票
                    price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
                    df_buy = df_buy.head(10)  # 只买最强的10个股票
                    orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
                        row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
                    pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
                  else:   # 下午两点之后锁仓
                    pass
                elif 0.33 <= ganzhou_index <=1: # 情绪高潮,全天不买入，开盘15分钟不涨停就卖出
                  if (now.hour==9 and now.minute > 45):
                     # 卖出信号生成,开盘15分钟不涨停就卖出一半仓位                    
                    sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
                    pub.sendMessage(str(TradeSignalTopic.SELL_HALF),message=sell_message)
            except Exception as e:
                print(f'东方财富接口调用异常:{e}')
            # 11、发送消息通知
            now = datetime.now()
            if now.hour >= 14: df = df[df["pct"] < 10] # 上午10点后，通知中会过滤掉涨幅大于10%的股票
            df = df.head(self.topN)
            df['name'] = df.apply(lambda row: "☀" + row['name'] if row['is_hot_industry'] else row['name'], axis=1)
            wecom_msg_enabled = os.environ.get('WECOM_MSG_ENABLED').lower() == 'true'
            if wecom_msg_enabled and df.shape[0] > 0:
              WeComNotification().send_stock_df(title=self.name, df=df, ganzhou_index=ganzhou_index)
