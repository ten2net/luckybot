from datetime import datetime
import math
from typing import List
from core.constants import Constants
from collector.akshare_data_collector import AkshareDataCollector
from core.topic import FavorSignalTopic, TradeSignalTopic
from filter.filter_chain import FilterChain
from filter.fund.fund_filter import NameFilter, SymbolFilter, TotalCapitalFilter
from filter.trading.amount_filter import AmountFilter
from filter.trading.turnover_filter import TurnoverFilter
from filter.trading.volume_filter import HighVolumeFilter
from notification.wecom import WeComNotification
from radar.base import StockRadar
from pool.pool import LargeBuyStockPool
from filter.trading.indictor_trading_filter import IndicatorTradingFilter
import os
from termcolor import colored
from pubsub import pub
from trader.base import OrderMessage

class LargeBuyStockRadar(StockRadar):
    def __init__(self,name:str="大笔买入", topN:int = 22):
        self.name = name
        self.topN = min(topN,22)
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
        mainStockPool = LargeBuyStockPool()        
        stockPools = [mainStockPool]  
        symbols = []
        for stockPool in stockPools:
            symbols += stockPool.get_symbols()
        # 3、先对symbols进行基本面过滤,以便减少后续计算量
        symbols_spot_df = market_spot_df[market_spot_df['code'].isin(symbols)]
        fand_filter_list = [SymbolFilter(),
                            NameFilter(),
                            TotalCapitalFilter(min_threshold=50, max_threshold=1200), #总市值过滤                            
                            ]
        fand_filter_chain = FilterChain(fand_filter_list)
        symbols_spot_df = fand_filter_chain.apply(symbols_spot_df)        
        symbols = symbols_spot_df['code'].tolist()
        # 4、获取股票数据，并附加其他指标
        # df = mainStockPool.get_data_with_indictores(symbols,withCDL=False)
        df = mainStockPool.get_data(symbols)
        df = df.merge(market_spot_df, on="code", how="left") 
        # 5、附加其他指标
        # 6、筛选股票，实现单独的过滤器，添加到过滤器链中即可
        filters = [
            AmountFilter(threshold=5), # 昨日成交额过滤器，过滤掉成交额太小的股票
            HighVolumeFilter(threshold=1.3), # 昨日成交量过滤器，过滤掉成交量大于5日均量1.3倍的股票
        ]
        filter_chain = FilterChain(filters)
        df = filter_chain.apply(df)                       
        
        df['upper_rate'] = df['code'].apply(lambda x: 20 if (
            x.startswith('3') or x.startswith('68')) else 10)

        df['space_limit'] = (
            df['upper_limit_y'] - df["close"]) / df['upper_limit_y']         
        
        if df.shape[0] == 0:
            print("无满足条件的股票！")
        else:
            # 计算情绪指数
            ganzhou_index =self.compute_market_sentiment_index()
            # 7、评分            
            # df['score'] = PriceScorer().score(df) + VolumeScorer().score(df) + PEScorer().score(df) +
            #               SentimentScorer().score(df)
            
            # 8、排序            
            df = df.head(round(1.5 * self.topN))  #自选股不受长度限制
            df['is_hot_industry'] = df['code'].apply(lambda code: akshareDataCollector.is_hot_industry(code))
            
            df.sort_values(by=['is_hot_industry','pct'], ascending=[False,False], inplace=True)
            df = df.reset_index(drop=True)
            print(colored(f"""{self.name}发现了 {df.shape[0]} 个目标：{df['name'].tolist()}""","green"))
            # 9、自选股
            results = df['code'].tolist()
            results = results[::-1]  # 确保新加自选的在上面
            favor_message={
              "group_name": self.name,
              "symbols": results
            }
            pub.sendMessage(str(FavorSignalTopic.UPDATE_FAVOR),message=favor_message)

            #10、交易信号生成，主程序中启动的模拟盘交易管理器SimTraderManagement负责侦听交易信号，实施交易
            # try:            
            #     now = datetime.now()
            #     if ganzhou_index < 0.03:   # 情绪太差,只卖不买
            #       if (now.hour==9 and now.minute <=45) or (now.hour==14 and now.minute <=10):   # 发出一键清仓信号
            #         pub.sendMessage(str(TradeSignalTopic.SELL_ALL),message=OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index))
            #       else:
            #         pub.sendMessage(str(TradeSignalTopic.SELL),message=OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)) 
            #     elif 0.05 <= ganzhou_index < 0.12: # 情绪一般,适度买卖
            #       if (now.hour==9 and now.minute <=45) :   # 开盘15分钟只买不卖
            #         if now.minute >10:
            #           # 卖出信号生成，不涨停就先卖出1/4                   
            #           sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
            #           pub.sendMessage(str(TradeSignalTopic.SELL_QUARTER),message=sell_message)                     
            #         # 买入信号生成
            #         df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 5)]  # 过滤掉当日上涨空间已经不多的股票                    
            #         df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
            #         price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
            #         df_buy = df_buy.head(5)  # 只买最强的5个股票
            #         orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
            #             row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
            #         pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
            #       elif (now.hour == 9 and now.minute > 45) or (9 < now.hour <= 14) :  # 下午两点之前只买不卖
            #         # 买入信号生成
            #         df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 3)]  # 过滤掉上涨空间还有三分之一的股票                         
            #         df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
            #         df_buy = df_buy[df_buy['pct'] < 5]  # 只买红票
            #         price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
            #         df_buy = df_buy.head(5)  # 只买最强的5个股票
            #         orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
            #             row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
            #         pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
            #       else:   # 下午两点之后只卖不买
            #         # 卖出信号生成
            #         sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
            #         pub.sendMessage(str(TradeSignalTopic.SELL),message=sell_message)
            #     elif 0.12 <= ganzhou_index < 0.22: # 情绪良好,多买少卖
            #       if (now.hour==9 and now.minute <=45) :   # 开盘15分钟只买不卖
            #         if now.minute >10:
            #           # 卖出信号生成，不涨停就先卖出1/2                   
            #           sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
            #           pub.sendMessage(str(TradeSignalTopic.SELL_HALF),message=sell_message)                     
            #         # 买入信号生成
            #         df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 5)]  # 过滤掉当日上涨空间已经不多的股票                    
            #         df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
            #         price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
            #         df_buy = df_buy.head(10)  # 只买最强的10个股票
            #         orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
            #             row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
            #         pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
            #       elif (now.hour == 9 and now.minute > 45) or (9 < now.hour <= 14) :  # 下午两点之前只买不卖
            #         # 买入信号生成
            #         df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 3)]  # 过滤掉上涨空间还有三分之一的股票                         
            #         df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
            #         df_buy = df_buy[df_buy['pct'] < 5]  # 只买红票
            #         price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
            #         df_buy = df_buy.head(10)  # 只买最强的10个股票
            #         orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
            #             row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
            #         pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
            #       else:   # 下午两点之后只卖不买
            #         # 卖出信号生成                    
            #         sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
            #         pub.sendMessage(str(TradeSignalTopic.SELL),message=sell_message)
            #     elif 0.22 <= ganzhou_index < 0.33: # 情绪很好,只买不卖
            #       if (now.hour==9 and now.minute <=45) :   # 开盘15分钟
            #         if now.minute >10:
            #           # 卖出信号生成，不涨停就先卖出                   
            #           sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
            #           pub.sendMessage(str(TradeSignalTopic.SELL),message=sell_message)                     
            #         # 买入信号生成
            #         df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 5)]  # 过滤掉当日上涨空间已经不多的股票                    
            #         df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
            #         price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
            #         df_buy = df_buy.head(10)  # 只买最强的10个股票
            #         orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
            #             row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
            #         pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
            #         # 卖出信号生成，不涨停就卖                   
            #         sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
            #         pub.sendMessage(str(TradeSignalTopic.SELL_HALF),message=sell_message)                    
            #       elif (now.hour == 9 and now.minute > 45) or (9 < now.hour <= 14) :  # 下午两点之前只买不卖
            #         # 买入信号生成
            #         df_buy = df[(df['upper_rate'] - df['pct']) > (df['upper_rate'] / 3)]  # 过滤掉上涨空间还有三分之一的股票                         
            #         df_buy = df_buy[df_buy['close'] > df_buy['open']]  # 只买红票
            #         df_buy = df_buy[df_buy['pct'] < 5]  # 只买红票
            #         price_rate = 1 + ganzhou_index / 100  # 价格调整比例,情绪越好，挂价越高于现价，情绪越差，挂价越低于现价
            #         df_buy = df_buy.head(10)  # 只买最强的10个股票
            #         orderMessages = [OrderMessage(strategyName=self.name, symbol=row['code'], price=min(round(
            #             row['close'] * price_rate, 2), row['upper_limit_y']), pct=row['pct'], index=ganzhou_index) for index, row in df_buy.iterrows()]
            #         pub.sendMessage(str(TradeSignalTopic.BATCH_BUY), message=orderMessages) 
            #       else:   # 下午两点之后锁仓
            #         pass
            #     elif 0.33 <= ganzhou_index <=1: # 情绪高潮,全天不买入，开盘15分钟不涨停就卖出
            #       if (now.hour==9 and now.minute > 45):
            #          # 卖出信号生成,开盘15分钟不涨停就卖出一半仓位                    
            #         sell_message = OrderMessage(strategyName=self.name, symbol=None, price=None, pct=None ,index=ganzhou_index)
            #         pub.sendMessage(str(TradeSignalTopic.SELL_HALF),message=sell_message)
            # except Exception as e:
            #     print(f'东方财富接口调用异常:{e}')            
            # 11、发送消息通知  
            now = datetime.now()
            if now.hour == 9 and now.minute<= 40:  # 只在开盘5分钟内发送此类消息 
                df = df.head(self.topN)   
                df['name'] =  df.apply(lambda row: "☀"+ row['name'] if row['is_hot_industry'] else row['name'], axis=1)     
                wecom_msg_enabled= os.environ.get('WECOM_MSG_ENABLED').lower() == 'true'
                if wecom_msg_enabled and df.shape[0] > 0:
                    WeComNotification().send_stock_df(title=self.name, df=df, ganzhou_index=ganzhou_index)

