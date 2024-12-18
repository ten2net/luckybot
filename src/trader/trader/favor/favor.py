from datetime import datetime
import math
from typing import List, Optional
from urllib.parse import urlencode

from dotenv import load_dotenv
from favor.base import Favor
import favor.em_favor_api as em
import requests
from requests import Response, Session
import os
import json
import time
import demjson3
import logging

logger = logging.getLogger(__name__)

class FavorForEM(Favor):
    def __init__(self, appkey:str, token:str, **kwargs):
        self.appkey= appkey
        self.session = requests.Session()  
        self.HEADER = {
            'Referer':'https://quote.eastmoney.com/zixuan/',
            'Cookie':token
        }   

    def __parse_resp__(self,resp: Response, key=None):
        if resp.status_code != 200:
            raise Exception(f"code:{resp.status_code},msg:{resp.content}")
 
        result = resp.text
        js_text = result[result.index("(") + 1: result.index(")")]

        ret = demjson3.decode(js_text)
        logger.info(f"ret:{ret}")
        data = ret.get("data")
        if data and key:
            result_value = data.get(key)
        else:
            result_value = data

        resp.close()
        return ret["state"], result_value   
    def __build_url__(self, action:str, params: dict) -> str:
        ts = int(time.time() * 1000)  - 10
        url = f"http://myfavor.eastmoney.com/v4/webouter/{action}?appkey={self.appkey}&cb=jQuery112407703233916827181_{ts}&"
        url += urlencode(params) +"&"
        url += f"&_={ts}"
        url = url.replace("0%25","0%") # 还原urlencode错误转义的em股票代码
        url = url.replace("1%25","1%") # 还原urlencode错误转义的em股票代码
        return url     
    def get_groups(self):
        params: dict = {
            "g": 1
        }
        url = self.__build_url__(action="ggdefstkindexinfos",params=params)
        resp = self.session.get(url, headers=self.HEADER)
        _, value = self.__parse_resp__(resp, key="ginfolist")
        return value

    def create_group(self, group_name:str):
        params: dict = {
            "gn":group_name
        }
        url = self.__build_url__(action="ag",params=params)
        resp = self.session.get(url, headers=self.HEADER)
        _, value = self.__parse_resp__(resp)
        return value


    def rename_group(self, group_id, group_name):
        params: dict = {
            "g":group_id,
            "gn":group_name
        }
        url = self.__build_url__(action="mg",params=params)
        resp = self.session.get(url, headers=self.HEADER)
        ret, _ = self.__parse_resp__(resp)
        return ret    


    def del_group(self, group_name=None, group_id=None):
        if not group_id:
            assert group_name is not None
            group_id = self.get_group_id(group_name)
            if not group_id:
                raise Exception(f"could not find group:{group_name}")
        params: dict = {
            "g":group_id
        }
        url = self.__build_url__(action="dg",params=params)
        resp = self.session.get(url, headers=self.HEADER)
        ret, _ = self.__parse_resp__(resp, key=None)
        return ret 

    def get_group_id(self,group_name):
        groups = self.get_groups()
        groups = [group for group in groups if group["gname"] == group_name]
        if groups:
            return groups[0]["gid"]
        return None
    def get_symbols(self, group_name="自选股") -> List[str]:
        return self.__list_entities__(group_name=group_name)
    def __list_entities__(self, group_name=None, group_id=None):
        if not group_id:
            assert group_name is not None
            group_id = self.get_group_id(group_name)
            if not group_id:
                # raise Exception(f"could not find group:{group_name}")
                print(f"could not find group:{group_name}")
                return []
        params: dict = {
            "g":group_id
        }
        url = self.__build_url__(action="gstkinfos",params=params)
        resp = self.session.get(url, headers=self.HEADER)
        
        _, result = self.__parse_resp__(resp)
        datas = result["stkinfolist"]
        return [data["security"].split("$")[1] for data in datas]
    def del_from_group(self,code, entity_type="stock", group_name=None, group_id=None):
        if not group_id:
            assert group_name is not None
            group_id = self.get_group_id(group_name)
            if not group_id:
                raise Exception(f"could not find group:{group_name}")
        code = self.to_eastmoney_code(code, entity_type=entity_type)
        params: dict = {
            "g":group_id,
            "sc":code
        }
        url = self.__build_url__(action="ds",params=params)
        resp = self.session.get(url, headers=self.HEADER)

        return self.__parse_resp__(resp)
    def add_symbols_to_group(self, 
            codes, entity_type="stock", group_name=None, group_id=None
    ):
        if not group_id:
            assert group_name is not None
            group_id = self.get_group_id(group_name)
            if not group_id:
                raise Exception(f"could not find group:{group_name}")
        codeList = [self.to_eastmoney_code(code, entity_type=entity_type) for code in codes]
        codeList = ",".join(codeList)

        params: dict = {
            "g":group_id,
            "scs":codeList
        }
        url = self.__build_url__(action="aslot",params=params)
        resp = requests.get(url, headers=self.HEADER)
        # print(resp.text)
    def del_symbols_from_group(self, 
            codes, entity_type="stock", group_name=None, group_id=None
    ):
        if not group_id:
            assert group_name is not None
            group_id = self.get_group_id(group_name)
            if not group_id:
                raise Exception(f"could not find group:{group_name}")
        codeList = [self.to_eastmoney_code(code, entity_type=entity_type) for code in codes]
        codeList = ",".join(codeList)

        params: dict = {
            "g":group_id,
            "scs":codeList
        }
        url = self.__build_url__(action="dslot",params=params)
        resp = self.session.get(url, headers=self.HEADER)

        return self.__parse_resp__(resp)
    def del_all_from_group(self, entity_type="stock", group_name=None, group_id=None):
        if not group_id:
            assert group_name is not None
            group_id = self.get_group_id(group_name)
            if not group_id:
                raise Exception(f"could not find group:{group_name}")
        codes = self.__list_entities__(group_name)
        split_codes = [codes[i:i + 45] for i in range(0, len(codes), 45)]
        for batch in split_codes: 
            codeList = [self.to_eastmoney_code(code, entity_type=entity_type) for code in batch]
            codeList = ",".join(codeList)
            
            params: dict = {
                "g":group_id,
                "scs":codeList
            }
            url = self.__build_url__(action="dslot",params=params)
            resp = requests.get(url, headers=self.HEADER)

    def add_to_group(self, 
            code, entity_type="stock", group_name=None, group_id=None
    ):
        if not group_id:
            assert group_name is not None
            group_id = self.get_group_id(group_name)
            if not group_id:
                raise Exception(f"could not find group:{group_name}")
        code = self.to_eastmoney_code(code, entity_type=entity_type)

        params: dict = {
            "g":group_id,
            "sc":code
        }
        url = self.__build_url__(action="as",params=params)
        resp = self.session.get(url, headers=self.HEADER)

        return self.__parse_resp__(resp)
    def update_favor(self, symbols: List[str] = [], group_name: str = "斐纳斯精选",daily=False):
        self.__update_favor_list__(
            symbols, group_full_name=f"{group_name[:3]}全榜", group_new_name=group_name,daily=daily)
    def __update_favor_list__(self, symbol_list:list[str],group_full_name:str ,group_new_name:str,daily=False):
        # 添加到东方财富自选股
        # group_new_name中只包含最新的出票
        if not daily:
            group_id_new = self.get_group_id(group_new_name)
            if not group_id_new: 
                self.create_group(group_new_name)
            else:
                # 删除上一榜出票
                self.del_all_from_group( group_name=group_new_name, entity_type="stock")  
        # group_full_name中包含全部的出票
        group_id_full = self.get_group_id(group_full_name)
        if not group_id_full:          
            self.create_group(group_full_name) 
        else:
            self.del_all_from_group( group_name=group_full_name, entity_type="stock")  
            # pass
            # now = datetime.now()
            # if now.hour< 9 or (now.hour== 9 and now.minute < 26):   # 删除前一天的出票
            #     self.del_all_from_group( group_name=group_full_name, entity_type="stock")  
           
        # 添加自选 
        group_name_list =[group_full_name] if daily else [group_new_name,group_full_name] 
        # group_name_list =[group_new_name,group_full_name] 
        for group_name in group_name_list:       
            self.add_symbols_to_group(symbol_list, group_name=group_name, entity_type="stock")
            
    def to_eastmoney_code(self, code, entity_type="stock"):
        if entity_type == "stock":
            code_ = int(code)
            # 上海
            if 600000 <= code_ <= 800000:
                return f"1%24{code}"
            else:
                return f"0%24{code}"
        if entity_type == "block":
            return f"90${code}"
        if entity_type == "stockhk":
            return f"116%24{code}"
        if entity_type == "stockus":
            return f"105%24{code}"
        assert False
