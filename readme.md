# 揽宝 (LuckyBot) - 基于ROS 2的多机器人股票自动化交易系统

[![GitHub Release](images/luckybot.svg)](https://github.com/ten2net/luckybot/releases) 选择揽宝，让智能交易成为您的财富增长引擎。
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2](https://img.shields.io/badge/ROS%202-jazzy-blue.svg)](https://index.ros.org/packages/jazzy/)

揽宝（LuckyBot）是一个创新的基于ROS 2的多机器人股票自动化交易系统，旨在为投资者提供一个全面、高效和自动化的交易解决方案。

## 特性

- **多机器人协同**：集成用户交互、账户管理和投资顾问服务。
- **实时交易执行**：精确执行交易操作，确保合规性和安全性。
- **市场分析与预测**：利用先进的数据分析技术提供市场趋势预测。
- **用户友好的界面**：直观的操作体验，轻松设置交易参数和监控账户状态。

## 系统架构

揽宝（LuckyBot）由以下三个核心组件构成：

1. **用户交互机器人（UserBot）**：处理用户请求和指令。
2. **账户管理机器人（AccountBot）**：执行交易操作并监控账户活动。
3. **投资顾问机器人（AdvisorBot）**：提供市场分析和投资策略。

## 安装指南

在开始之前，请确保您已经安装了ROS 2 jazzy和必要的依赖项。以下是安装揽宝（LuckyBot）的步骤：

```bash
# 克隆仓库
git clone https://github.com/ten2net/luckybot.git
cd luckybot

# 安装依赖项
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# 构建项目
colcon build

## 使用方法

启动系统前，请确保您的ROS 2环境已经设置好：

# 源环境
source install/setup.bash

# 启动系统
ros2 launch luckybot launch/start.launch.py
```

## 文档

更多详细的使用说明和API文档，请参阅[揽宝文档](https://ten2net.github.io/luckybot/docs)。

## 贡献

我们欢迎任何形式的贡献，包括代码、文档和bug报告。请查看我们的[贡献指南](CONTRIBUTING.md)以了解更多信息。

## 许可证

揽宝（LuckyBot）遵循[Apache 2.0许可证](LICENSE)。请查看[许可证文件](LICENSE)以了解更多信息。

## 联系

- **邮件**：[ten2net@163.com](mailto:ten2net@163.com)
- **社区**：加入我们的[社区论坛](https://www.e-u.cn)以获取支持和分享经验。