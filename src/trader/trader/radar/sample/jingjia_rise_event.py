from core.constants import Constants
from collector.akshare_data_collector import AkshareDataCollector
from core.topic import FavorSignalTopic
from filter.filter_chain import FilterChain
from filter.fund.fund_filter import NameFilter, SymbolFilter, TotalCapitalFilter
from filter.trading.amount_filter import AmountFilter
from filter.trading.turnover_filter import TurnoverFilter
from filter.trading.volume_filter import HighVolumeFilter
from kline.kline_style import KLineStyles, KLineStyles_cdls
from notification.wecom import WeComNotification
from radar.base import StockRadar
from pool.pool import AmountStockPool, FavorStockPool, JingJiaRiseStockPool, LargeBuyStockPool
from filter.trading.indictor_trading_filter import IndicatorTradingFilter
import os
from pubsub import pub
from termcolor import colored

class JingJiaRiseStockRadar(StockRadar):
    def __init__(self,name:str="竞价上涨", topN:int = 22):
        self.name = name
        self.topN = min(topN,22)

    def startup(self):
        # 1 准备市场行情快照，获取股票代码、名称、总市值、PE、PB等指标，用于筛选股票
        market_spot_df = AkshareDataCollector().get_stock_zh_a_spot_em()
        market_spot_df = market_spot_df[Constants.SPOT_EM_COLUMNS_BASE] 
        # 2、准备主股票池和其他附加股票池
        mainStockPool = JingJiaRiseStockPool()        
        stockPools = [mainStockPool]  
        symbols = []
        for stockPool in stockPools:
            symbols += stockPool.get_symbols(k = 20)
        # 3、先对symbols进行基本面过滤,以便减少后续计算量
        symbols_spot_df = market_spot_df[market_spot_df['code'].isin(symbols)]
        fand_filter_list = [SymbolFilter(),
                            NameFilter(),
                            TotalCapitalFilter(min_threshold=20, max_threshold=1200), #总市值过滤
                            ]
        fand_filter_chain = FilterChain(fand_filter_list)
        symbols_spot_df = fand_filter_chain.apply(symbols_spot_df)        
        symbols = symbols_spot_df['code'].tolist()
        # 4、获取股票数据，并附加其他指标
        df = mainStockPool.get_data(symbols)
        df = df.merge(market_spot_df, on="code", how="left")        
        
        # 5、附加其他指标
        # 6、筛选股票，实现单独的过滤器，添加到过滤器链中即可
        filters = [
            # AmountFilter(threshold=2), # 昨日成交额过滤器，过滤掉成交额小于2亿的股票
            HighVolumeFilter(threshold=2.5), # 昨日成交量过滤器，过滤掉成交量大于5日均量1.3倍的股票
        ]
        filter_chain = FilterChain(filters)
        df = filter_chain.apply(df)
        
        if df.shape[0] == 0:
            print("无满足条件的股票！")
        else:
            # 7、评分            
            # df['score'] = PriceScorer().score(df) + VolumeScorer().score(df) + PEScorer().score(df) +
            #               SentimentScorer().score(df)
            
            # 8、排序            
            df = df.head(round(2 * self.topN))  #自选股不受长度限制
            print(colored(f"""{self.name}发现了 {df.shape[0]} 个目标：{df['name'].tolist()}""","green"))
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
            
            # 10、发送消息通知  
            df = df.head(self.topN)          
            wecom_msg_enabled= os.environ.get('WECOM_MSG_ENABLED').lower() == 'true'
            if wecom_msg_enabled:
                ganzhou_index =self.compute_market_sentiment_index()
                WeComNotification().send_stock_df(title=self.name, df=df, ganzhou_index=ganzhou_index)
