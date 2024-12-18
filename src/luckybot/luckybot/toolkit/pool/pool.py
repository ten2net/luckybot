
from typing import List
import pandas as pd
from .base import StockPool

from ..collector.akshare_data_collector import AkshareDataCollector

class AmountStockPool(StockPool):
  def __init__(self):
    self.adc = AkshareDataCollector()
  def get_symbols(self,cloumn_name:str="amount",k:int=100) -> List[str]:
    """
    获取股票符号列表，默认为按成交额排序的前100只股票
    
    Args:
        cloumn_name (str, optional): 用于排序的列名，默认为'amount'。
        k (int, optional): 返回的股票数量，默认为100。
    
    Returns:
        list: 包含股票符号的列表。
    
    """
    df = self.adc.get_stock_zh_a_spot_em()
    df.sort_values(by=cloumn_name, ascending=False, inplace=True)
    df = df.head(k).copy()
    symbols = df['code'].tolist()
    return symbols  
  def get_data_frame(self,cloumn_name:str="amount",k:int=100) -> pd.DataFrame:
    """
    获取股票符号列表，默认为按成交额排序的前100只股票
    
    Args:
        cloumn_name (str, optional): 用于排序的列名，默认为'amount'。
        k (int, optional): 返回的股票数量，默认为100。
    
    Returns:
        pd.DataFrame: 包含股票数据。
    
    """
    df = self.adc.get_stock_zh_a_spot_em()
    df.sort_values(by=cloumn_name, ascending=False, inplace=True)
    return df.head(k).copy()
  
