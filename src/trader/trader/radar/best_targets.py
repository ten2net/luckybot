from datetime import datetime
import math
from typing import List
from core.constants import Constants
from collector.akshare_data_collector import AkshareDataCollector
from core.topic import FavorSignalTopic, TradeSignalTopic
from filter.filter_chain import FilterChain
from filter.fund.fund_filter import NameFilter, SymbolFilter, TotalCapitalFilter
from filter.trading.amount_filter import AmountFilter
from filter.trading.volume_filter import HighVolumeFilter
from notification.wecom import WeComNotification
from radar.base import StockRadar
from pool.pool import ATPStockPool, AmountStockPool, BidAskStockPool, HotRankStockPool, HotSymbolStockPool
import os
from termcolor import colored
from pubsub import pub
from trader.base import OrderMessage

class BestTargetStockRadar(StockRadar):
    def __init__(self, name: str = "封神榜", k:int=200, n: int = 30):
        self.name = name
        self.k = k 
        self.n = n 

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
        mainStockPool = BidAskStockPool(k=self.k,n=self.n)
        stockPools = [mainStockPool]     
        symbols = []
        for stockPool in stockPools:
            symbols += stockPool.get_symbols()
        # 3、先对symbols进行基本面过滤,以便减少后续计算量
        symbols_spot_df = market_spot_df[market_spot_df['code'].isin(symbols)]
        # 使用 reindex 方法重新排序 symbols_spot_df
        symbols_spot_df = symbols_spot_df.set_index('code').reindex(symbols).reset_index()        
        fand_filter_list = [SymbolFilter(),
                            NameFilter(),
                            TotalCapitalFilter(min_threshold=100, max_threshold=1500),  # 总市值过滤
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
        # 6、筛选股票，实现单独的过滤器，添加到过滤器链中即可
        filters = [
            AmountFilter(threshold=10),  # 昨日成交额过滤器，过滤掉成交额太小的股票
            HighVolumeFilter(threshold=2), # 昨日成交量过滤器，过滤掉成交量大于5日均量2倍的股票
        ]
        filter_chain = FilterChain(filters)
        df = filter_chain.apply(df)
        # print("filters=",  len(df))
        # 9、自选股
        now = datetime.now()
        dt =now.strftime("%Y-%m-%d-%H-%M-%S") 
        save_path = "results/bests/{}.csv".format(dt)
        directory = os.path.dirname(save_path)
        if not os.path.exists(directory):
            os.makedirs(directory)   
        # print(df.columns) 
        df["date"] = now.strftime("%Y-%m-%d %H:%M:00")   
        df = df[['date', 'code', 'name', 'open', 'close', 'high', 'low', 'volume', 'amount', 'amp',
       'pct', 'turnover', 'close_yestday', 'upper_limit_x', 'lower_limit_x',
       'pct_yestday', 'volume_yestday', 'amount_yestday', 'turnover_yestday',
       'volume_3_with_today', 'volume_3', 'pe', 'pb',
       'total_capital', 'circulating_capital', '60_day_pct', 'ytd_pct',
       'upper_limit_y', 'lower_limit_y']]
        df.to_csv(save_path, index=False,encoding='utf_8_sig')
                                   
        results = df['code'].tolist()
        results = results[::-1]  # 确保新加自选的在上面

        split_results = [results[i:i + 45] for i in range(0, len(results), 45)] # 每次最多添加46个，否则接口报Connection reset by peer错误
        for batch in split_results: 
            favor_message={
            "group_name": self.name,
            "symbols": batch,
            "daily":True
            }
            pub.sendMessage(str(FavorSignalTopic.UPDATE_FAVOR),message=favor_message)

