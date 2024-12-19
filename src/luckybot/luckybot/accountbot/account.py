#!/usr/bin/env python3
from luckybot.toolkit.trader.sim.trader import Trader
import rclpy
import json
from std_msgs.msg import String
from luckybot.base import LuckyBot

class AccountHelper:
    def __init__(self,userid, sim_token):
        self.userid = userid
        self.sim_token = sim_token

    def fetch_accounts(self) -> dict:
        from urllib.parse import urlencode
        import requests
        import time        
        def build_url( params: dict, base=None) -> str:
            url = self.base_url if base is None else base
            ts = int(time.time() * 1000) - 10
            url += f"{ts}&"
            url += f"userid={self.userid}&"
            url += f"plat=2&ver=web20&"
            url += urlencode(params) + "&"
            url += f"r=9633477&_={ts}"
            return url
        order_type = "spo_zuhe_preview"  # spo_bal_info 账户信息
        params: dict = {
            "type": order_type,
            "sylType": "10001"
        }
        url = build_url(
            params, base="https://simqry2.eastmoney.com/qry_tzzh_v2?cb=jQuery35109976387848607984_1722654955385")
        hd = {
            "Referer": "https://i.eastmoney.com/",
            "Cookie": self.sim_token
        }
        resp = requests.get(url, headers=hd)
        if resp.status_code == 200:
            result = resp.text
            js_text = result[result.index("(") + 1: result.index(")")]
            ret = json.loads(js_text)
            accounts = ret.get("data")

            return [{"accountNo": data['zjzh'],
                     "accountName": data['zuheName'],
                     "return_total": data['zsyl'],
                    "return_today": data['syl_dr'],
                     "return_5d": data['syl_5r'],
                     "return_20d": data['syl_20r'],
                     "return_6d": data['syl_60r'],
                     "return_250d": data['syl_250r'],
                     "win": data['dealWinCnt'],
                     "fail": data['dealfailCnt'],
                     "pos": data['holdPos']}
                    for data in accounts
                    ]
        else:
            result = resp.text
            js_text = result[result.index("(") + 1: result.index(")")]
            ret = json.loads(js_text)
            msg = ret.get("message")
            print(f'{msg} . {url}')
            return []       

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
        self.get_logger().info(self.username + "------" + self.account_name + ' : ' + self.strategt_mapping)
        
        accountHeler = AccountHelper(self.userid, self.sim_token)
        account_return_info_list = accountHeler.fetch_accounts()
        account_return_info = [item for item in account_return_info_list if item.get('accountName') == self.account_name]
        self.__dict__.update(account_return_info[0])
        # self.get_logger().info(json.dumps(account_return_info_list))
        self.get_logger().info(account_return_info_list[0]["accountName"])
        self.get_logger().info(self.accountNo)
        
        self.trader: Trader = Trader(self.userid, self.accountNo, self.sim_token)
        balance_info = self.trader.get_balance_info()
        self.__dict__.update(balance_info)
        self.get_logger().info(json.dumps(balance_info))
        pos = self.trader.get_position()
        self.get_logger().info(json.dumps(pos))
        groups = self.trader.get_groups()
        self.get_logger().info(json.dumps(groups, ensure_ascii=False))
        # 10: 这是发布者的质量服务（Quality of Service，QoS）设置。QoS设置定义了消息传递的可靠性和历史保持策略。在这里，10是QoS配置的整数表示，对应于rmw_qos_profile_sensor_data，这是一个适用于传感器数据的QoS配置，它提供了一定的可靠性保证和历史保持策略。
        # self.publisher_ = self.create_publisher(String, 'luckybot/accountbot', 10)
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