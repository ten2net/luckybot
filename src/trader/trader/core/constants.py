class Constants:
    BAR_DAY_COLUMNS = ["date","open", "close", "high",
                       "low", "volume", "amount", "amp", "pct", "turnover"]
    BAR_MINUTE_COLUMNS = ["date", "open", "close", "high",
                          "low", "volume", "amount", "amplitude", "pct", "turnover"]
    TA_INDIECT_NAMES = ["cmf", "roc", "sma", "ema", "obv", "slope"]
    TA_INDIECT_LENTHES = [3, 5, 10]
    SPOT_EM_COLUMNS = [
        "code",
        "name",
        "close",
        "pct",
        "volume",
        "amount",
        "amp",
        "high",
        "low",
        "open",
        "close_yesterday",
        "volume_ratio",
        "turnover",
        "pe",
        "pb",
        "total_capital",
        "circulating_capital",
        "minute_5_change",
        "day_60_pct",
        "ytd_pct"
    ]
    SPOT_EM_COLUMNS_BASE = [
        "code",
        "name",
        "pe",
        "pb",
        "total_capital",
        "circulating_capital",
        "60_day_pct",
        "ytd_pct",
        "upper_limit",
        "lower_limit"
    ]
