#!/usr/bin/env python3
   
import rclpy
from rclpy.node import Node
import pandas as pd
from std_msgs.msg import String
from luckybot_interface.msg import StockSpotData,StockSpotDataArray

class Account(Node):
    def __init__(self):
        super().__init__('account')
        self.subscription = self.create_subscription(
            StockSpotDataArray,
            'market/spot', 
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        data_list = []
        ignore_fields =['_check_fields', '_header']
        for stock_data in msg.data:
            # print(stock_data.__slots__)
            data ={}
            for attr in stock_data.__slots__:  # 使用 __slots__ 获取消息字段名
                if attr in ignore_fields: continue
                field = attr[1:]                
                data[field] = getattr(stock_data, field)
            data_list.append(data)
        self.market_spot_df  = pd.DataFrame(data_list)
        self.get_logger().info(f'收到行情数据： {self.market_spot_df.shape}')  
                          

def main(args=None):
    try:
        rclpy.init(args=args)
        node = Account()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass        
    except Exception as e:
        print(f"Exception: {e}")        

if __name__ == '__main__':
    main()      