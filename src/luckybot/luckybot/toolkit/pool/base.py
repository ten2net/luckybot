from abc import ABC, abstractmethod
from typing import List
import pandas as pd
from tqdm import tqdm

import warnings

class StockPool(ABC):
    @abstractmethod
    def get_symbols(self) -> List[str]:
        pass
    def get_data(self, symbols: List[str])->pd.DataFrame:
        """
        从ADC数据源获取最新的数据。
        
        Args:
            symbols：股票代码列表
        
        Returns:
            pd.DataFrame: 包含最新数据的DataFrame，每列对应一个symbol的最新数据。
        
        """
         
        pbar = tqdm(range(len(symbols)), desc=f'正在获取最新行情数据进行因子计算...',
                    bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed} < {remaining}, {rate_fmt}]', colour='yellow')    
        # 初始化一个空列表来存储最后一行的数据
        last_rows_data = []
        
        # 遍历symbols，获取每个symbol的最后一行数据
        for symbol in symbols:
            df_symbol = self.adc.get_data(symbol)
            if not df_symbol.empty:  # 确保DataFrame不是空的
                last_row = df_symbol.iloc[-1:].copy()  # 获取最后一行并复制
                last_row['code'] = symbol
                last_rows_data.append(last_row)
            pbar.update(1)
        
        # 使用pd.concat合并最后一行的DataFrame列表
        return pd.concat(last_rows_data)   if len(last_rows_data) > 0 else pd.DataFrame()     

    def get_data_with_indictores(self, symbols: List[str],withCDL: bool=False)->pd.DataFrame:
        """
        获取每个symbol带有指标的最新行情数据。
        
        Args:
            symbols：股票代码列表
        
        Returns:
            pd.DataFrame: 包含所有symbol带有指标的最新行情数据的DataFrame。
            DataFrame的列包括原始数据和symbol列，其中symbol列用于标识每行数据对应的symbol。
        
        """
        warnings.filterwarnings('ignore')
        pbar = tqdm(range(len(symbols)), desc=f'正在获取最新行情数据进行因子计算...',
                    bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed} < {remaining}, {rate_fmt}]', colour='yellow')    
        # 初始化一个空列表来存储最后一行的数据
        last_rows_data = []
        
        # 遍历symbols，获取每个symbol的最后一行数据
        for symbol in symbols:
            df_symbol = self.adc.get_data_with_indictores(symbol)
            if not df_symbol.empty:  # 确保DataFrame不是空的
                if withCDL:  
                   add_cdl_pattern(df_symbol) # 添加K线形态CDL指标
                last_row = df_symbol.iloc[-1:].copy()  # 获取最后一行并复制
                last_row['code'] = symbol
                last_rows_data.append(last_row)
            pbar.update(1)
        
        # 使用pd.concat合并最后一行的DataFrame列表
        df_last_rows = pd.concat(last_rows_data, ignore_index=True)
        return df_last_rows
    
