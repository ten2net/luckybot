#!/usr/bin/env python3
   
import rclpy
import pandas as pd
from luckybot_interface.msg import StockSpotDataArray
from luckybot.base import LuckyBot
from luckybot.core.constants import Constants

class DemoBot(LuckyBot):
    def __init__(self):
        super().__init__('account')
        self.declare_parameter('interval', 1)
        # 获取参数
        self.index_interval = self.get_parameter('interval').get_parameter_value().integer_value
        self.timer = self.create_timer(self.index_interval, self.timer_callback)

    def timer_callback(self):
      if self.index is None: return
      if self.market_spot_df is None: return
      self.get_logger().info(f'index = {self.index}')   
      self.get_logger().info(f'market_spot_df.shape = {self.market_spot_df.shape}')          

def main(args=None):
    try:
        rclpy.init(args=args)
        node = DemoBot()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass        
    except Exception as e:
        print(f"Exception: {e}")        

if __name__ == '__main__':
    main()      