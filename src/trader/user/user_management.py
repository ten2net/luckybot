from typing import List
from dotenv import load_dotenv
import pandas as pd
from core.topic import FavorSignalTopic
import favor.em_favor_api as em
import requests
import os
import json

from pubsub import pub
from trader.sim.trader import Trader
from user.user import User

class UserManagement:
    def __init__(self):
        self.session = requests.Session()
        load_dotenv()
        user_config_list = json.loads(os.environ.get('USER_CONFIG_LIST'))    
        self.users =[User(user_config) for user_config in user_config_list ]  

    def get_users(self):
        return self.users
    def on_update_favor_signal(self, message: dict):
        daily=message.get('daily', False)
        for user in self.users:
            user.favor.update_favor(symbols=message['symbols'],group_name=message['group_name'],daily=daily)
    def startWatch(self):
      pub.subscribe(self.on_update_favor_signal, str(FavorSignalTopic.UPDATE_FAVOR))    

