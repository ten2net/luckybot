from abc import ABC, abstractmethod
import pandas as pd
class StockFilter(ABC):
    @abstractmethod
    def filter(self, df:pd.DataFrame)->pd.DataFrame:
        pass
