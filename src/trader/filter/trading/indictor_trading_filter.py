from ..base import StockFilter
import pandas as pd

class IndicatorTradingFilter(StockFilter):
    def __init__(self, indicator_name: str, threshold: float, comparison_operator: str = '>='):
        self.indicator = indicator_name
        self.threshold = threshold
        # 定义比较运算符的映射
        self.comparison_operators = {
            '>': lambda x, y: x > y,
            '>=': lambda x, y: x >= y,
            '<': lambda x, y: x < y,
            '==': lambda x, y: x == y,
            '<=': lambda x, y: x <= y,
        }
        # 确保提供的比较运算符是有效的
        if comparison_operator not in self.comparison_operators:
            raise ValueError(f"Invalid comparison operator: {comparison_operator}")
        self.comparison_operator = self.comparison_operators[comparison_operator]

    def filter(self, df:pd.DataFrame)->pd.DataFrame:
        # 使用lambda函数和比较运算符的映射来生成条件
        condition = self.comparison_operator(df[self.indicator], self.threshold)
        return df[condition]
