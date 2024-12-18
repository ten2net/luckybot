# from radar.cci_88 import CCIStockRadar
from radar.everyday_targets import EverydayTargetStockRadar
from radar.hot_symbols import HotSymbolStockRadar
from radar.best_targets import BestTargetStockRadar

# from radar.jingjia_rise_event import JingJiaRiseStockRadar
from radar.large_buy_event import LargeBuyStockRadar
from radar.sample.cci_88 import CCIStockRadar
from radar.sample.jingjia_rise_event import JingJiaRiseStockRadar
from radar.sample.ta_kline_style import KLineStyleStockRadar
# from radar.ta_kline_style import KLineStyleStockRadar
# 后期维护主要工作：
# 继承StockRadar，实现特定筛选策略和排序策略的金融雷达系统,
# 部署到下面的数组stock_radares中，然后多线程启动每个雷达
all_radares = [
    CCIStockRadar(name="斐纳斯强势300", cci_threshold=250),
    KLineStyleStockRadar(name="K线异常", threshold=0, topN =200),
    CCIStockRadar(name="斐纳斯强势300", cci_threshold=300),
    LargeBuyStockRadar(name="大笔买入", topN =22),
    HotSymbolStockRadar(name="热股强势", topN =22),
    JingJiaRiseStockRadar(name="竞价上涨", topN =22),
]
stock_radares = [
    # LargeBuyStockRadar(name="大笔买入", topN =22),
    # HotSymbolStockRadar(name="热股强势", topN =22),
    EverydayTargetStockRadar(name="每日情绪榜",topN=100),
    BestTargetStockRadar(name="封神榜",k=300,n=40)
]
jinjia_stock_radares = [
    HotSymbolStockRadar(name="热股强势", topN =22),
    LargeBuyStockRadar(name="大笔买入", topN =22),
]
dev_stock_radares = [
    # HotSymbolStockRadar(name="热股强势", topN =22),
    LargeBuyStockRadar(name="大笔买入", topN =22),
]
 