#!/usr/bin/env python3
from .strategy.base import StrategyManager
from .strategy.sell_on_consecutive_declines import SellOnConsecutiveDeclines
from .trader.sim.trader import Trader
import rclpy
import json
import time
from std_msgs.msg import String
from ..base import LuckyBot
from collections import deque


class AccountBot(LuckyBot):
    def __init__(self,name):
        super().__init__('luckybot_accountbot')
        # self.__dict__.update(user_config)
        self.declare_parameter('username', '')
        self.declare_parameter('userid', '')
        self.declare_parameter('em_token', '')
        self.declare_parameter('sim_token', '')
        self.declare_parameter('name', '')
        self.declare_parameter('strategt_mapping', '') 
        self.declare_parameter('em_appkey', '')       
        # 获取参数
        self.username = self.get_parameter('username').get_parameter_value().string_value        
        self.userid = self.get_parameter('userid').get_parameter_value().string_value        
        self.em_token = self.get_parameter('em_token').get_parameter_value().string_value        
        self.sim_token = self.get_parameter('sim_token').get_parameter_value().string_value        
        self.account_name = self.get_parameter('name').get_parameter_value().string_value        
        self.strategt_mapping = self.get_parameter('strategt_mapping').get_parameter_value().string_value        
        self.em_appkey = self.get_parameter('em_appkey').get_parameter_value().string_value        
        # self.get_logger().info(self.username + "------" + self.account_name + ' : ' + self.strategt_mapping)
        
        self.strategy_manager = StrategyManager()
        self.strategy_manager.register_strategy('consecutive_declines', SellOnConsecutiveDeclines(threshold=2))  
        # TODO: adregisterd other strategies
        # 用参数控制该账户要启用的是指管理策略。逗号分隔的策略名
        self.declare_parameter('enabled_strategies', '')
        # 创建一个定时器来定期检查参数值
        # self.timer = self.create_timer(1.0, self.check_parameters) 
        group = Trader(self.userid, '', self.sim_token).get_group(self.account_name)
        # self.get_logger().info(json.dumps(group, ensure_ascii=False))
        self.trader: Trader = Trader(self.userid, group.get("groupNo"), self.sim_token)
        # balance_info = self.trader.get_balance_info()
        # self.__dict__.update(balance_info)
        # self.get_logger().info(json.dumps(balance_info, ensure_ascii=False))
        # pos = self.trader.get_position()
        # self.get_logger().info(json.dumps(pos, ensure_ascii=False,indent=4))
        
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.index_history = deque(maxlen=1000)
      
    def check_parameters(self):
        current_enabled_strategies = self.get_parameter('enabled_strategies').get_parameter_value().value
        if current_enabled_strategies != self.previous_enabled_strategies:
            self.previous_enabled_strategies = current_enabled_strategies
            enabled_strategies = current_enabled_strategies.split(',')
            for strategy in enabled_strategies:
                self.strategy_manager.enable_strategy(strategy.strip())
            for strategy in list(self.strategy_manager.enabled_strategies):
                if strategy not in enabled_strategies:
                    self.strategy_manager.disable_strategy(strategy)
            self.get_logger().info(f'Enabled strategies updated: {enabled_strategies}')        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'enabled_strategies':
                enabled_strategies = param.value.split(',')
                for strategy in enabled_strategies:
                    self.strategy_manager.enable_strategy(strategy.strip())
                for strategy in list(self.strategy_manager.enabled_strategies):
                    if strategy not in enabled_strategies:
                        self.strategy_manager.disable_strategy(strategy)

    def execute_strategies(self):
        account_data ={
            "position" : self.trader.get_position(),
            "balance": self.trader.get_balance_info(),
        }
        # self.get_logger().info(json.dumps(account_data, ensure_ascii=False))
        self.strategy_manager.execute_strategies(self.market_data, account_data, self.trader)
    def timer_callback(self):
        # self.get_logger().info(json.dumps(self.market_data, ensure_ascii=False,indent=4))
        if self.market_data.get("index") is None: return
        if self.market_spot_df is None: return  
        if not self.index_history or self.market_data.get("index") != self.index_history[-1].get("index"):
            self.index_history.append({
                "timestamp" : int(time.time()),
                "index": self.market_data.get("index")
            })
        self.get_logger().info(json.dumps(list(self.index_history), ensure_ascii=False))             
        if self.market_data.get('index') < 0.05:
            self.execute_strategies()
def main(args=None):
    try:
        rclpy.init(args=args)
        node = AccountBot("")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass        
    except Exception as e:
        print(f"Exception: {e}")

if __name__ == '__main__':
    main()    