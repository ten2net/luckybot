#!/usr/bin/env python3
   
from typing import Dict
import json
import rclpy
from rclpy.node import Node
import pandas as pd
from std_msgs.msg import String
from luckybot_interface.msg import StockSpotDataArray
from std_msgs.msg import Float32
from std_msgs.msg import Header
from luckybot.core.constants import Constants

class LuckyBot(Node):
    def __init__(self,name):
        super().__init__(name)
        
        self._market_data = {}
        self._market_spot_df = None
        
        self.index_subscription = self.create_subscription(
            Float32,
            f'{Constants.TOPIC_NAME_PREFIX}/index', 
            self.index_callback,
            10)
        self.spot_subscription = self.create_subscription(
            StockSpotDataArray,
            f'{Constants.TOPIC_NAME_PREFIX}/spot', 
            self.spot_callback,
            10)
        
    def index_callback(self, msg):
        self._market_data.update({'index': round(msg.data,3)})
    def spot_callback(self, msg):
        data_list = []
        ignore_fields =['_check_fields', '_header']
        for stock_data in msg.data:
            data ={}
            for attr in stock_data.__slots__:  # 使用 __slots__ 获取消息字段名
                if attr in ignore_fields: continue
                field = attr[1:]                
                data[field] = getattr(stock_data, field)
            data_list.append(data)
        self._market_spot_df  = pd.DataFrame(data_list)
    @property
    def market_spot_df(self) -> pd.DataFrame:
        """Get market spot."""
        return self._market_spot_df        
    @property
    def market_data(self) -> Dict:
        """Get market index."""
        return self._market_data          
                          

def main(args=None):
    try:
        rclpy.init(args=args)
        node = LuckyBot(f"{Constants.NODE_NAME_PREFIX}_luckybot")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass        
    except Exception as e:
        print(f"Exception: {e}")        

if __name__ == '__main__':
    main()      