from abc import ABC, abstractmethod

class DataCollector(ABC):
    @abstractmethod
    def get_data(self, symbol, start_date, end_date):
        """
        Fetch data from the source.
        :param symbol: The symbol of the stock.
        :param start_date: The start date for fetching data.
        :param end_date: The end date for fetching data.
        :return: Data fetched from the source.
        """
        pass

    @abstractmethod
    def fetch_intraday_data(self, symbol, start_time, end_time):
        """
        Fetch intraday data from the source.
        :param symbol: The symbol of the stock.
        :param start_time: The start time for fetching intraday data.
        :param end_time: The end time for fetching intraday data.
        :return: Intraday data fetched from the source.
        """
        pass
    def calculate_limit_prices(self,yesterday_close, limit_ratio):
        """
        根据昨日收盘价和涨跌停幅度计算今日的涨停价和跌停价。
        
        :param yesterday_close: 昨日收盘价
        :param limit_ratio: 涨跌停幅度比例（例如，10% 应传入 0.10）
        :return: 涨停价和跌停价
        """
        upper_limit_price = round(yesterday_close * (1 + limit_ratio),2)
        lower_limit_price = round(yesterday_close * (1 - limit_ratio),2)
        return upper_limit_price, lower_limit_price    