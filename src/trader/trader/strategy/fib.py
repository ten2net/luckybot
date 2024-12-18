import akshare as ak
import pandas as pd

# 基于股票近期的下跌趋势，当股票达到 38.2% 的斐波那契回调水平时，您买入股票，期望该股票会反弹并进一步上涨。 
# 基于股票近期的上升趋势，您在 61.8% 的斐波那契回调水平下方设置止损指令，以便在股价跌破该点时将损失降到最低。 
# 基于股票近期的上升趋势，您在 161.8% 的斐波那契扩展水平上设定了获利订单，以捕捉先前股价走势的整个回调。

# 斐波那契适用于各周期，从年线直至分钟K线图；适用于包括大A在内的股票、商品、货币等标的物。
# 23.6%用于红三兵或三只乌鸦（或四根连续的阳线或阴线），连接三根K线的高低点，下一根回撤到23.6%较大概率行情延续。
# 38.2%在弱势回调时只到此位置；在谐波形态中经常用于判断是否满足条件，即一段回撤必须到达38.2%。
# 50%重要位置，仅次于61.8%且基本相当，一段行情回撤的正中间。
# 61.8%最重要位置，附近一般较大概率反转；在谐波形态中判断和区分形态的重要位置，如加特利必须到此位置，而蝙蝠不到此位置。
# 78.6%在谐波中的重要位置，如加特利形态的D点很可能在此位置出现反转。
# 88.6%在谐波中的重要位置，如蝙蝠形态中的D点反转位置；做为极限反转位置，使用者多已由谐波中重要位置转为非谐波中同样非常重要的位置。

# fig, ax = plt.subplots(figsize=(15,5))

# ax.plot(df1.Date, df1.Price)

# ax.axhspan(level1, Price_Min, alpha=0.4, color='lightsalmon')
# ax.axhspan(level2, level1, alpha=0.5, color='palegoldenrod')
# ax.axhspan(level3, level2, alpha=0.5, color='palegreen')
# ax.axhspan(Price_Max, level3, alpha=0.5, color='powderblue')

# plt.ylabel("Price")
# plt.xlabel("Date")

# plt.title('Fibonacci')

# plt.show()

# 定义斐波那契回撤策略类
class FibonacciRetracement:
    def __init__(self, high, low, levels=None):
        self.high = high
        self.low = low
        self.levels = levels if levels else [ 0.236, 0.382, 0.5, 0.618, 0.786, 1, 1.618,2.618]

    def generate_signal(self, current_price):
        if not hasattr(self, 'retracement_levels'):
            differences = self.high - self.low
            self.retracement_levels = [self.low + differences * level for level in self.levels]
        
        levels = self.retracement_levels
        # print(self.high , self.low,levels)
        if current_price <= levels[1]:  # Buy signal at 0.236 retracement level
            return 'buy'
        elif current_price >= levels[-2]:  # Sell signal at 0.786 retracement level
            return 'sell'
        else:
            return 'hold'
    def nearest_support_resistance(self, price)->tuple[float, float]:
        differences = self.high - self.low
        if not hasattr(self, 'retracement_levels'):
            self.retracement_levels = [self.low + differences * level for level in self.levels]
        
        levels = self.retracement_levels        
        price_difference = self.high - self.low
        
        
        # 计算斐波那契回撤水平作为阻力位
        fib_23_6 = self.high - (price_difference * 0.382)
        fib_38_2 = self.high - (price_difference * 0.500)
        fib_50 = self.high - (price_difference * 0.618)
        
        # 计算斐波那契扩展水平作为支撑位
        fib_61_8 = self.low + (price_difference * 1.618)
        fib_100 = self.low + price_difference
        fib_161_8 = self.low + (price_difference * 2.618)
        
        # 将所有可能的支撑和阻力位存储在列表中
        levels = [fib_23_6, fib_38_2, fib_50, fib_61_8, fib_100, fib_161_8]
        
        # 找出离当前价格最近的阻力位
        nearest_resistance = min(levels, key=lambda x: (x - price)**2)        
        # 找出离当前价格最近的支撑位
        nearest_support = max(levels, key=lambda x: (price - x)**2)        
        return nearest_resistance, nearest_support      

# 定义VWAP计算类
class VWAPCalculator:
    def __init__(self):
        self.cumulative_price_volume_sum = 0
        self.cumulative_volume = 0

    def update(self, price, volume):
        self.cumulative_price_volume_sum += price * volume
        self.cumulative_volume += volume

    def calculate(self):
        return (self.cumulative_price_volume_sum / self.cumulative_volume) if self.cumulative_volume else None

class FibonacciTradingSignal:
    def __init__(self, high, low):
        self.high = high  # 最高价
        self.low = low    # 最低价
        # 斐波那契回撤和扩展比率
        ratios = [0.236, 0.382, 0.500, 0.618, 0.786, 1.000, 1.236, 1.382, 1.5, 1.618, 1.786, 2.000, 2.618]
        price_diff = self.high - self.low
        self.fib_levels = [round(self.low + price_diff * ratio,3) for ratio in ratios] 
 
    def generate_signal(self,currentPrice: float, threshold=0.02,ganzhou_index: float=0.0):
        # 获取最近的三个阻力位和支撑位
        resistances = sorted([level for level in self.fib_levels if level > currentPrice])[:3]
        supports = sorted([level for level in self.fib_levels if level < currentPrice], reverse=True)[:3]
        resistances=[self.high, self.high * 1.236,self.high * 1.382] if len(resistances) == 0 else  resistances
        resistances=[round(esistanc,3)  for esistanc in resistances]        
        supports=[self.low, self.low * 0.786, self.low * 0.618 ] if len(supports) == 0 else  supports
        supports=[round(support,3)  for support in supports]
        distances = [abs(currentPrice - level) for level in self.fib_levels]
        ind = distances.index(min(distances))
        closest_level = self.fib_levels[ind]
        closest_degree = min(distances) / closest_level 
        # print(ind,closest_level,min(distances),closest_degree)
        # 确定信号类型
        buy_ind_max = 3  # if ganzhou_index> 0.1 else 2
        if closest_degree < threshold:
            if 0 <= ind <= buy_ind_max:
                signal = "buy"
            elif 5<= ind < 8:
                signal = "sell" 
            else:
                signal = "hold"  # 未到水平，持有
        else:
            if ind < 2:
                signal = "buy"  # 到最低水平，买入
            elif ind > 6:
                signal = "sell"  # 到最高水平，卖出
            else:
                signal = "hold"  # 未到水平，持有
        return (signal, resistances, supports)

class FibonacciTradingSignal4:
    def __init__(self, high, low):
        self.high = high  # 最高价
        self.low = low    # 最低价
        # 斐波那契回撤和扩展比率
        ratios = [0.236, 0.382, 0.500, 0.618, 0.786, 1.000, 1.236, 1.382, 1.5, 1.618, 1.786, 2.000, 2.618]
        price_diff = self.high - self.low
        self.fib_levels = [round(self.low + price_diff * ratio,3) for ratio in ratios] 
 
    def generate_signal(self,currentPrice: float, threshold=0.02,ganzhou_index: float=0.0):
        # 获取最近的三个阻力位和支撑位
        resistances = sorted([level for level in self.fib_levels if level > currentPrice])[:3]
        supports = sorted([level for level in self.fib_levels if level < currentPrice], reverse=True)[:3]
        resistances=[self.high, self.high * 1.236,self.high * 1.382] if len(resistances) == 0 else  resistances
        resistances=[round(esistanc,3)  for esistanc in resistances]        
        supports=[self.low, self.low * 0.786, self.low * 0.618 ] if len(supports) == 0 else  supports
        supports=[round(support,3)  for support in supports]
        distances = [abs(currentPrice - level) for level in self.fib_levels]
        ind = distances.index(min(distances))
        closest_level = self.fib_levels[ind]
        closest_degree = min(distances) / closest_level 
        # print(ind,closest_level,min(distances),closest_degree)
        # 确定信号类型
        buy_ind_max = 6  # if ganzhou_index> 0.1 else 2
        if closest_degree < threshold:
            if 0 <= ind <= buy_ind_max:
                signal = "buy"
            elif 6<= ind < 8:
                signal = "sell" 
            else:
                signal = "hold"  # 未到水平，持有
        else:
            if ind < (buy_ind_max - 1):
                signal = "buy"  # 到最低水平，买入
            elif ind >= 6:
                signal = "sell"  # 到最高水平，卖出
            else:
                signal = "hold"  # 未到水平，持有
        return (signal, resistances, supports)

