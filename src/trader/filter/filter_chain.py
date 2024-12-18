from .base import StockFilter
from typing import List
class FilterChain:
    def __init__(self,filters:List[StockFilter]=[]):
        self.filters = filters

    def add_filter(self, filter:StockFilter):
        self.filters.append(filter)

    def apply(self, df):
        for filter in self.filters:
            df = filter.filter(df)
        return df