#!/bin/bash

# 检查是否提供了足够的参数
if [ "$#" -ne 2 ]; then
    echo "用法: $0 <package_name> <node_name>"
    exit 1
fi

# 接收参数作为环境变量
PKG_NAME=$1
NODE_NAME=$2
NODE_MODULE=${2,,} # 将NODE_MODULE转换为小写

# 创建Node文件
NODE_FILE_PUB="src/${PKG_NAME}/${PKG_NAME}/${NODE_NAME}Publisher.py"
NODE_FILE_SUB="src/${PKG_NAME}/${PKG_NAME}/${NODE_NAME}Subscriber.py"

# 使用sed命令替换模板中的占位符
# sed "s/{{NODE_NAME}}/${NODE_NAME}/g" node_template.py > "$NODE_FILE"
# sed "s/{{PKG_NAME}}/${PKG_NAME}/g" node_template.py > "$NODE_FILE"
sed -e "s/{{NODE_NAME}}/${NODE_NAME}/g" -e "s/{{PKG_NAME}}/${PKG_NAME}/g" node_template_publisher.py > "$NODE_FILE_PUB"
sed -e "s/{{NODE_NAME}}/${NODE_NAME}/g" -e "s/{{PKG_NAME}}/${PKG_NAME}/g" node_template_subscriber.py > "$NODE_FILE_SUB"

echo "Node file '$NODE_FILE_PUB' has been created."
echo "Node file '$NODE_FILE_SUB' has been created."

# 更新setup.py文件
SETUP_PY="src/${PKG_NAME}/setup.py"

# 使用sed添加新的console_script条目
sed -i "/'console_scripts': \[/a \\
            'pub_$NODE_MODULE = ${PKG_NAME}.${NODE_NAME}Publisher:main'," "$SETUP_PY"
sed -i "/'console_scripts': \[/a \\
            'sub_$NODE_MODULE = ${PKG_NAME}.${NODE_NAME}Subscriber:main'," "$SETUP_PY"

echo "Updated '$SETUP_PY' with new console script entry for $NODE_NAME."