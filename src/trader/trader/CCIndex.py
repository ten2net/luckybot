import string
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from luckybot_interface.msg import AmazingQuote
from std_msgs.msg import Float32

import pandas as pd
import akshare as ak

from custom_msg.my_custom_msg import SampleClass, sample_function_for_square_of_sum
# from entity_controller.msg import Person
from .collector.akshare_data_collector import AkshareDataCollector
from .pool.pool import AmountStockPool

class TurtleNode(Node):
    def __init__(self):
        super().__init__('TurtleNode')  # 使用固定的节点名称以避免与launch文件中的参数冲突
        
        # msg = Person()
        # msg.name = "ROS User"
        # msg.age = 4  
        # print(">>>>>>>>>>>>>",msg)      
        # 声明参数
        self.declare_parameter('node_name', 'trader')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.2)
        # 获取参数
        self.node_name = self.get_parameter('node_name').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        # self.get_logger().info(f">>>>>>>>{self.node_name} {self.linear_speed} {self.angular_speed}")
        
        # 使用参数设置发布者
        self.publisher_ = self.create_publisher(Float32, f'{self.node_name}/ccindex', 10)
        self.timer = self.create_timer(15, self.timer_callback)


    def timer_callback(self):
        msg = Float32()
        # msg.header.stamp = self.get_clock().now()  # ROS 2时间戳
        msg.data = self.compute_market_sentiment_index()
        # msg.data =  random.uniform(0, 1)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f"' % msg.data)

    def compute_market_sentiment_index(self) -> float:
        stockPool = AmountStockPool()
        df =stockPool.get_data_frame(cloumn_name="amount", k=300)       

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
    rclpy.init(args=args)
    turtlenode = TurtleNode()
    rclpy.spin(turtlenode)
    turtlenode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
