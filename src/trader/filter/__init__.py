from filter.trading.amount_filter import AmountFilter
from filter.trading.pct_filter import PCTFilter
from filter.trading.turnover_filter import TurnoverFilter
from filter.trading.volume_filter import HighVolumeFilter, VolumeFilter
from .fund.fund_filter import SymbolFilter,NameFilter,PEFilter,TotalCapitalFilter,CirculatingCapitalFilter
from .trading.indictor_trading_filter import IndicatorTradingFilter

def get_filters():
    return [SymbolFilter,
            NameFilter,
            PEFilter,
            TotalCapitalFilter,
            CirculatingCapitalFilter, 
            IndicatorTradingFilter,
            HighVolumeFilter,
            VolumeFilter,
            TurnoverFilter,
            PCTFilter,
            AmountFilter]
