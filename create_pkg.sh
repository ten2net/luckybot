#!/bin/bash

# 检查是否提供了足够的参数
if [ "$#" -ne 2 ]; then
    echo "用法: $0 <package_name> <node_name>"
    exit 1
fi

# 接收参数作为环境变量
PKG_NAME=$1
NODE_NAME=$2
EXE_NAME=$2


# 检查node-name是否包含非法字符
if [[ ! $NODE_NAME =~ ^[a-zA-Z0-9_]+$ ]]; then
    echo "Error: node-name can only contain alphanumeric characters and underscores."
    exit 1
fi

# 创建包目录并进入src目录
# mkdir -p src/$PKG_NAME
# cd src/$PKG_NAME

# 创建ROS2 Python包
ros2 pkg create $PKG_NAME --destination-directory ./src --build-type ament_python --dependencies rclpy --node-name $NODE_NAME

# cd ../..
# 创建.gitignore文件并添加内容
if [ -f .gitignore ]; then
    echo ".gitignore file exists."
else
    touch .gitignore
    echo "build/" >> .gitignore
    echo "install/" >> .gitignore
    echo "log/" >> .gitignore
fi


# # 创建build.sh文件并添加内容
# touch build.sh
# echo "#!/bin/bash" > build.sh
# echo "" >> build.sh
# echo "# Build the package" >> build.sh
# echo "colcon build --packages-select $PKG_NAME" >> build.sh
# echo "" >> build.sh
# echo "# Source the setup script" >> build.sh
# echo ". install/setup.bash" >> build.sh

# # 添加可执行权限
# chmod +x build.sh

# 初始化git仓库并提交
if [ -d .git ]; then
    echo "This is a Git repository."
else
    git init .
    git add .
    git commit -m "pkg create $PKG_NAME $NODE_NAME"
fi


# 构建包
# ./build.sh
colcon build
. install/setup.bash 

# 执行节点
ros2 run $PKG_NAME $EXE_NAME