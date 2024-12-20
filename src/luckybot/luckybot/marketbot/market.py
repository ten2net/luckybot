import string
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32
from std_msgs.msg import Header

import pandas as pd
import akshare as ak

from luckybot.toolkit.collector.akshare_data_collector import AkshareDataCollector
from luckybot.toolkit.pool.pool import AmountStockPool

from luckybot_interface.msg import StockSpotData,StockSpotDataArray
from luckybot.core.constants import Constants

def dataframe_to_stockspotdata(df):
    """
    Convert a DataFrame to a list of StockSpotData messages.

    :param df: pandas DataFrame, the output of akshare.stock_zh_a_spot_em()
    :return: list of StockSpotData messages
    """
    stock_data_array = StockSpotDataArray()
    stock_data_array.header = Header()   
    stockspotdata_list=[]
    for index, row in df.iterrows():
        # print(index)
        stockspotdata = StockSpotData()
        for attr in stockspotdata.__slots__:  # 使用 __slots__ 获取消息字段名
            field = attr[1:]
            # print(attr,field)
            if field in df.columns:  # 检查DataFrame中是否存在对应的列                
                setattr(stockspotdata, field, row[field])  # 将DataFrame列值赋给消息字段
        stockspotdata_list.append(stockspotdata)
    stock_data_array.data = stockspotdata_list 
    return stock_data_array

class MarketBot(Node):
    def __init__(self, name):
        super().__init__(f'{Constants.TOPIC_NAME_PREFIX}_{name}')
        self.declare_parameter('index_interval', 15)
        self.declare_parameter('spot_interval', 15)
        self.declare_parameter('top_n', 300)
        # 获取参数
        self.index_interval = self.get_parameter('index_interval').get_parameter_value().integer_value
        self.spot_interval = self.get_parameter('spot_interval').get_parameter_value().integer_value
        self.top_n = self.get_parameter('top_n').get_parameter_value().integer_value
        # 使用参数设置发布者
        self.index_publisher = self.create_publisher(Float32, f'{Constants.TOPIC_NAME_PREFIX}/index', 10)
        self.spot_publisher = self.create_publisher(StockSpotDataArray, f'{Constants.TOPIC_NAME_PREFIX}/spot', 10)
        
        self.index_timer = self.create_timer(self.index_interval, self.index_timer_callback)
        self.spot_timer = self.create_timer(self.spot_interval, self.spot_timer_callback)
    def index_timer_callback(self):
        msg = Float32()
        # msg.data = round(self.compute_market_sentiment_index() + random.uniform(0, 1), 3)
        msg.data = round( random.uniform(-0.1, 0.1), 3)
        self.index_publisher.publish(msg)
    def spot_timer_callback(self):
        df = self.get_market_spot()
        msg = dataframe_to_stockspotdata(df)
        self.spot_publisher.publish(msg)
        
        
    def get_market_spot(self) -> pd.DataFrame:
        akshareDataCollector = AkshareDataCollector()
        market_spot_df = akshareDataCollector.get_stock_zh_a_spot_em()  
        return market_spot_df       

    def compute_market_sentiment_index(self) -> float:
        stockPool = AmountStockPool()
        df =stockPool.get_data_frame(cloumn_name="amount", k = self.top_n)       

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

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MarketBot(f"{Constants.TOPIC_NAME_PREFIX}_marketbot")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass        
    except Exception as e:
        print(f"Exception: {e}")   

if __name__ == '__main__':
    main()
