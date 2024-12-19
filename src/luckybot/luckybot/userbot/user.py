#!/usr/bin/env python3
from luckybot.toolkit.favor.favor import FavorForEM
import rclpy
import json
from std_msgs.msg import String
from luckybot.base import LuckyBot

class UserBot(LuckyBot):
    def __init__(self,name):
        # super().__init__('luckybot_userbot')
        super().__init__(name)
        self.declare_parameter('username', '')
        self.declare_parameter('userid', '')
        self.declare_parameter('em_token', '')
        self.declare_parameter('sim_token', '')
        self.declare_parameter('em_appkey', 'wwwwww')
        # 获取参数
        self.username = self.get_parameter('username').get_parameter_value().string_value        
        self.userid = self.get_parameter('userid').get_parameter_value().string_value        
        self.em_token = self.get_parameter('em_token').get_parameter_value().string_value        
        self.sim_token = self.get_parameter('sim_token').get_parameter_value().string_value        
        self.em_appkey = self.get_parameter('em_appkey').get_parameter_value().string_value        
        self.favor: FavorForEM = FavorForEM(appkey=self.em_appkey, token=self.em_token)
        # symbols = self.favor.get_symbols("封神榜全榜")
        # self.get_logger().info(json.dumps(symbols, indent=4))
        # 10: 这是发布者的质量服务（Quality of Service，QoS）设置。QoS设置定义了消息传递的可靠性和历史保持策略。在这里，10是QoS配置的整数表示，对应于rmw_qos_profile_sensor_data，这是一个适用于传感器数据的QoS配置，它提供了一定的可靠性保证和历史保持策略。
        # self.publisher_ = self.create_publisher(String, 'luckybot/userbot', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2! I am {self.get_name()}'
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = UserBot("aa")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass        
    except Exception as e:
        print(f"Exception: {e}")

if __name__ == '__main__':
    main()    