import rclpy

import pandas as pd
import akshare as ak
from ..base import Market

class SellAllNode(Market):
    def __init__(self):
        super().__init__('sell_all')
        self.declare_parameter('interval', 1)
        # 获取参数
        self.index_interval = self.get_parameter('interval').get_parameter_value().integer_value
        
        self.timer = self.create_timer(self.index_interval, self.sell_all)
    def sell_all(self):
      if self.index is None: return
      if self.market_spot_df is None: return
      self.get_logger().info(f'index = {self.index}')   
      self.get_logger().info(f'market_spot_df.shape = {self.market_spot_df.shape}')   
      if self.index and self.index > 80:
          self.get_logger().info(f'开始卖出所有股票')
          # TODO: 卖出所有股票

def main(args=None):
    try:
        rclpy.init(args=args)
        node = SellAllNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass        
    except Exception as e:
        print(f"Exception: {e}")       

if __name__ == '__main__':
    main()
