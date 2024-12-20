import os
import launch
from launch import LaunchDescription,LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    package_name = "luckybot"
    share_directory = get_package_share_directory(package_name)
    # 定义参数
    user_config_file_arg = DeclareLaunchArgument(
        'user_config_file', default_value=os.path.join(
        share_directory, 'config', 'user_config.yaml'))
    
    def run_node_action(context: LaunchContext):
      user_config_file = context.launch_configurations['user_config_file']       
      with open(user_config_file, 'r') as file:
          user_config = yaml.safe_load(file)
          em_appkey =  user_config['em_appkey']
          print(em_appkey)

      # 创建用户节点和账户节点的列表
      user_nodes = []
      account_nodes = []

      for user in user_config['users']:
          # 创建用户节点
          user_node = Node(
              package=package_name,
              executable='user',              
              name='user_node_' + user['username'],
              output='screen',
              parameters=[{
                'username': user['username'],
                'userid': str(user['userid']),
                'em_token': user['em_token'],
                'sim_token': user['sim_token'],
                'em_appkey': em_appkey                
                }]
          )
          user_nodes.append(user_node)

          # 为每个用户下的每个账户创建账户节点
          account_index = 0
          for account in user['accounts']:
              account_index += 1
              account["id"] = f'{user['username']}_{account_index}'
              account_node = Node(
                  package=package_name,
                  executable='account',
                  name='account_node_' + account["id"],
                  output='screen',
                  parameters=[{
                    'username': user['username'],
                    'userid': str(user['userid']),
                    'em_token': user['em_token'],
                    'sim_token': user['sim_token'],
                    # 'em_appkey': em_appkey,
                    **account
                  }]
              )
              account_nodes.append(account_node)

      return [*user_nodes, *account_nodes]
    run_node_action_func = OpaqueFunction(function=run_node_action)


    # 创建并返回LaunchDescription
    ld = LaunchDescription([
        user_config_file_arg,   
        run_node_action_func
    ])

    return ld