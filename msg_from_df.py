import pandas as pd
import akshare as ak

from trader.collector.akshare_data_collector import AkshareDataCollector

def generate_msg_from_dataframe(df):
    """
    Generate the content of a ROS 2 message file based on the columns and data types of a DataFrame.

    :param df: pandas DataFrame
    :return: str, the content of the ROS 2 message file
    """
    lines = ["# StockSpotData.msg\n"]
    lines.append("std_msgs/Header header\n")
    for column in df.columns:
        field_type = get_ros_type(df[column].dtype)
        lines.append(f"{field_type} {column}")
    
    return "\n".join(lines)

def get_ros_type(dtype):
    """
    Map pandas data types to ROS 2 message field types.

    :param dtype: pandas data type
    :return: str, the corresponding ROS 2 message field type
    """
    if dtype == 'int64':
        return 'int64'
    elif dtype == 'float64':
        return 'float64'
    elif dtype == 'object':  # Assuming string type for object
        return 'string'
    elif dtype == 'bool':
        return 'bool'
    else:
        raise ValueError(f"Unsupported data type: {dtype}")


akshareDataCollector = AkshareDataCollector()
df = akshareDataCollector.get_stock_zh_a_spot_em()  

# Generate the message content
msg_content = generate_msg_from_dataframe(df)
print(msg_content)