import math
import os
from typing import List, Literal
import demjson3
from dotenv import load_dotenv
import requests
from urllib.parse import urlencode
import time

from account.trading_account import Account
from ..favor.base import Favor
from ..favor.favor import FavorForEM
from trader.sim.trader import Trader
class User:
    def __init__(self, user_config: dict):
        self.__dict__.update(user_config)
        load_dotenv()
        appkey = os.environ.get('EM_APPKEY')
        self.favor:Favor = FavorForEM(appkey=appkey, token=self.em_token)

    def get_sim_token(self):
        return self.sim_token

    def get_userid(self):
        return self.userid

    def get_username(self):
        return self.username

    def get_phone_no(self) -> dict:
        return self.phone_no

    def get_accounts(self) -> dict:
      accounts =[]
      account_return_info_list = self.__fetch_accounts__()
      for account_return_info in account_return_info_list:
          trader = Trader(self.userid,account_return_info["accountNo"],self.sim_token) 
          balance_info = trader.get_balance_info()
          
          account = Account(account_return_info,balance_info)            
          account.set_trader(trader)  # 把trader对象设置到account中,方便以后用账户直接调用交易接口
          
          accounts.append(account)
      return accounts
    def __fetch_accounts__(self) -> dict:
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
            ret = demjson3.decode(js_text)
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
            ret = demjson3.decode(js_text)
            msg = ret.get("message")
            print(f'{msg} . {url}')
            return []        
