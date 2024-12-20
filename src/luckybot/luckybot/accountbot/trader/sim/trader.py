
import math
from typing import List, Literal
# import demjson3
import json
import requests
from urllib.parse import urlencode
import time

class Trader:
    def __init__(self, userid: str, groupno: str, token: str):
        self.userid = userid
        self.groupno = groupno
        self.header = {
            'Referer': 'https://group.eastmoney.com/room/index.html',
            'Cookie': token
        }

        self.base_url = " https://simoper.eastmoney.com/oper_tzzh_v2?cb=jQuery1123026493533723791596_"

    def build_url(self, params: dict, base=None) -> str:
        url = self.base_url if base is None else base 
        ts = int(time.time() * 1000) - 10
        url += f"{ts}&"
        url += f"userid={self.userid}&"
        if base is None:
            url += f"zjzh={self.groupno}&"
        url += f"plat=2&ver=web20&"
        url += urlencode(params) +"&"
        # for key, value in params.items():
        #     url += f"{key}={value}&"
        url += f"r=9633477&_={ts}"
        return url

    def buy(self, code: str, price: float, stock_num: int) -> str:
        return self.commit_buy_or_sell_order(code, price, stock_num, 'buy', order_type="spo_order")

    def sell(self, code: str, price: float, stock_num: int) -> str:
        return self.commit_buy_or_sell_order(code, price, stock_num, 'sell', order_type="spo_order")

    def commit_buy_or_sell_order(self, code: str, price: float, stock_num: int, buy_or_sell: Literal['buy', 'sell'] = 'buy', order_type: Literal['spo_order', 'spo_order_limit'] = "spo_order") -> str:
        mktCode = "1" if code.startswith("6") else "0"
        order_type = order_type  # spo_hold 持仓信息
        mmfx = "1" if buy_or_sell == 'buy' else "2"  # 1:买 2：卖
        mmfx_zh = "买入" if buy_or_sell == 'buy' else "卖出"  # 1:买 2：卖

        params: dict = {
            "type": order_type,
            "mmfx": mmfx,
            "mktCode": mktCode,
            "stkCode": code,
            "price":price,
            "wtsl":math.floor(stock_num)
        }
        url = self.build_url(params)
        resp = requests.get(url, headers=self.header)
        if resp.status_code == 200:
            result = resp.text
            js_text = result[result.index("(") + 1: result.index(")")]
            # ret = demjson3.decode(js_text)
            ret = json.loads(js_text)
            result = ret.get("result")
            if result == 0:
                return f"{mmfx_zh}{code}, 下单成功"
            else:
                msg = ret.get("message")
                return f"{mmfx_zh}{code}, 下单失败! 原因：{msg} {url}"
        else:
            return f"请求失败！状态码：{resp.status_code}.{resp.content},{url}"

    def create_account(self, groupName: str, desc: str="API", authority:Literal[0, 1] = 0) -> str:
        order_type = "spo_create_zuhe"  #创建投资组合
        params: dict = {
            "type": order_type,
            "zuheName": groupName,
            "comment": desc,
            "authority": authority
        }
        url = self.build_url(params)

        resp = requests.get(url, headers=self.header)
        print(resp.content)
        result = resp.text
        js_text = result[result.index("(") + 1: result.index(")")]
        # ret = demjson3.decode(js_text)
        ret = json.loads(js_text)
        result = ret.get("result")
        if result == 0:
            return f"创建投资组合成功。 {ret.get('message')}"
        else:
            msg = ret.get("message")
            return f"创建投资组合失败! 原因：{msg} {url}"

    def cancel_order(self, code: str, order_no: str) -> str:
        mktCode = "1" if code.startswith("6") else "0"
        wth = order_no
        order_type = "spo_cancel"  # 撤单
        mmfx = "1"

        params: dict = {
            "wth": wth,
            "stkCode": code,
            "type": order_type,
            "mktCode": mktCode,
            "mmfx": mmfx
        }
        url = self.build_url(params)
        resp = requests.get(url, headers=self.header)
        result = resp.text
        js_text = result[result.index("(") + 1: result.index(")")]
        # ret = demjson3.decode(js_text)
        ret = json.loads(js_text)
        result = ret.get("result")
        if result == 0:
            return "撤单成功"
        else:
            msg = ret.get("message")
            return f"撤单失败! 原因：{msg}  {url}"

    def get_can_cancel_order(self) -> List[dict]:
        order_type = "spo_orders_cancel"  # 挂单信息

        params: dict = {
            "type": order_type
        }
        url = self.build_url(params)
        resp = requests.get(url, headers=self.header)
        if resp.status_code == 200:
            result = resp.text
            js_text = result[result.index("(") + 1: result.index(")")]
            # ret = demjson3.decode(js_text)
            ret = json.loads(js_text)
            data = ret.get("data")
            msg = ret.get("message")
            data = [{"code": item['stkCode'], "name": item['stkName'], "orderType": item['orderType'],
                     "mmflag": item['mmflag'], "order_no": item['wth']} for item in data]
            return data
        else:
            result = resp.text
            # js_text = result[result.index("(") + 1: result.index(")")]
            # ret = demjson3.decode(result)
            ret = json.loads(result)            
            msg = ret.get("error")
            print(msg, url)
            return None

    def get_position(self):
        order_type = "spo_hold"  # spo_hold 持仓信息
        params: dict = {
            "type": order_type
        }
        url = self.build_url(params)
        resp = requests.get(url, headers=self.header)
        if resp.status_code == 200:
            result = resp.text
            js_text = result[result.index("(") + 1: result.index(")")]
            # ret = demjson3.decode(js_text)
            ret = json.loads(js_text)
            data = ret.get("data")
            msg = ret.get("message")
            data = [{"code": item['stkCode'], "name": item['stkName'], "quantity": int(
                item['zqsl']), "quantity_can_use": int(item['kysl']), "purchase_price": float(item['cbj'])} for item in data]
            return data
        else:
            result = resp.text
            js_text = result[result.index("(") + 1: result.index(")")]
            # ret = demjson3.decode(js_text)
            ret = json.loads(js_text)
            msg = ret.get("message")
            print(msg, url)
            return None

    def get_balance_info(self) -> dict:
  
        order_type = "spo_bal_info"  # spo_bal_info 账户信息
        params: dict = {
            "type": order_type
        }
        url = self.build_url(params)
        resp = requests.get(url, headers=self.header)
        if resp.status_code == 200:
            result = resp.text
            js_text = result[result.index("(") + 1: result.index(")")]
            print("js_text", js_text)
            # ret = demjson3.decode(js_text)
            ret = json.loads(js_text)
            data = ret.get("data")
            # [{'fdyk': '-839.17', 'fdykRat': '-0.30', 'kyye': '650368.12', 'mktVal': '348792.72', 'sdje': '74128.72', 'zhName': '', 'zjzh': '241990400000029517', 'zzc': '999160.83'}]
            data = [{"account": item['zjzh'], "total_money": float(item['zzc']), "account_pct": float(item['fdykRat']), "account_return": float(
                item['fdyk']), "market_value": float(item['mktVal']), "can_use_money": float(item['kyye']), "freeze_money": float(item['sdje'])} for item in data]
            return data[0]
        else:
            result = resp.text
            js_text = result[result.index("(") + 1: result.index(")")]
            # ret = demjson3.decode(js_text)
            ret = json.loads(js_text)
            msg = ret.get("message")
            print(msg, url)
            return None
    def get_group(self,groupName) -> dict:  
        groups = self.get_groups()  
        for group in groups:
            if group['groupName'] == groupName:
                return group    
    def get_groups(self) -> List[dict]:
        url =f"https://simqry2.eastmoney.com/qry_tzzh_v2?type=spo_zuhe_preview&plat=2&ver=web20&userid={self.userid}"
        resp = requests.get(url)
        if resp.status_code == 200:
            result = resp.text
            ret = json.loads(result)
            groups = ret.get("data")

            return  [{"groupNo": data['zjzh'],
                    "groupName": data['zuheName'],
                    "return_total": data['zsyl'],
                    "return_today": data['syl_dr'], 
                    "return_5d": data['syl_5r'], 
                    "return_20d": data['syl_20r'], 
                    "return_6d": data['syl_60r'], 
                    "return_250d": data['syl_250r'], 
                    "win": data['dealWinCnt'], 
                    "fail": data['dealfailCnt'], 
                    "pos": data['holdPos']} 
                    for data in groups
                    ]
        else:
            result = resp.text
            # print(result)
            # js_text = result[result.index("(") + 1: result.index(")")]
            # # ret = demjson3.decode(js_text)
            # ret = json.loads(js_text)
            # msg = ret.get("message")
            # print(msg)
            return None

    def execute_buy(self, stock_prices: dict, position_ratio:float):
        """
        执行买入操作的函数。        

        :param position_ratio: 仓位比例（例如0.1表示10%）
        :param stock_prices: 每只股票的当前价格列表（字典形式，键为股票名称或代码，值为价格）
        :return: 买入的股票及其数量
        """
        # 先撤销未成交的挂单
        # can_cancel_orderes =self.get_can_cancel_order()
        # if can_cancel_orderes is not None:
        #     for order in can_cancel_orderes:
        #         if order['mmflag'] == '0':
        #             print(self.cancel_order(code=order["code"],order_no=order["order_no"]))        
                
        if position_ratio <-1 or position_ratio > 1:
            raise ValueError("仓位比例必须在-1和1之间")
        
        balance_info = self.get_balance_info()  # {'account': '241990400000029517', 'total_money': '999160.83', 'account_pct': '-0.30', 'account_return': '-839.17', 'market_value': '348792.72', 'can_use_money': '650368.12', 'freeze_money': '74128.72'}
        # if 0 < position_ratio < 0.3 and (float(balance_info['market_value']) /float(balance_info['total_money'])) > 0.8:
        #     return {"msg": "已经超过7成，风控模块拒绝买入"}
        # if -0.1 < position_ratio < 0 and (float(balance_info['market_value']) /float(balance_info['total_money'])) > 0.5:
        #     return {"msg": "情绪太差, 仓位已经超过一半，风控模块拒绝买入"}
        # 计算单次买入金额
        buy_amount_per_stock = float(balance_info['can_use_money']) * position_ratio // len(stock_prices.items())
        
        # 计算每只股票的买入数量
        transactions = {}
        for stock, price in stock_prices.items():
            buy_quantity = math.floor(buy_amount_per_stock / price)
            buy_quantity = buy_quantity - (buy_quantity % 100)  # 确保是整手
            if buy_quantity > 0:
                result:str = self.buy(stock, price, buy_quantity)
                transactions[stock] = (price,buy_quantity,result)
        return transactions
