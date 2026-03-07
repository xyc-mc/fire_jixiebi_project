#!/bin/bash
workspace_dir="$(pwd)"
config_file="$workspace_dir/src/start/config/init.yaml"

if [ -f "$config_file" ]; then
    rm -f "$config_file"
    echo "已清理旧的配置文件"
fi

# 创建新的配置文件（ROS 2参数格式）
cat > "$config_file" << EOF
/**:
  ros__parameters:
    workspace_dir: "$workspace_dir"
    init_time: "$(date '+%Y-%m-%d %H:%M:%S')"
EOF

echo "全局参数文件已创建: $config_file"

# 执行colcon构建
colcon build