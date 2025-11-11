#!/bin/bash

echo "======================================"
echo "RViz 启动测试脚本"
echo "======================================"

# 检查 RViz 是否安装
echo "检查 RViz 安装状态..."
if ! command -v rviz &> /dev/null; then
    echo "❌ RViz 未安装！请安装 RViz:"
    echo "   sudo apt-get install ros-$ROS_DISTRO-rviz"
    exit 1
else
    echo "✅ RViz 已安装"
fi

# 检查配置文件是否存在
RVIZ_CONFIG="/workspace/ws_rl_motor/src/rl_motor_lidar/FAST_LIO/rviz_cfg/merge_lidar.rviz"
if [ -f "$RVIZ_CONFIG" ]; then
    echo "✅ RViz 配置文件存在: $RVIZ_CONFIG"
else
    echo "❌ RViz 配置文件不存在: $RVIZ_CONFIG"
    exit 1
fi

# 检查 roscore 是否运行
echo "检查 roscore 状态..."
if pgrep -x "roscore" > /dev/null || pgrep -x "rosmaster" > /dev/null; then
    echo "✅ roscore 正在运行"
else
    echo "⚠️  roscore 未运行，正在启动..."
    roscore &
    sleep 3
fi

echo ""
echo "======================================"
echo "启动选项："
echo "======================================"
echo "1. 启动 Livox 雷达融合 (默认启动 RViz)"
echo "2. 启动多雷达融合 (手动启动 RViz)"
echo "3. 仅启动 RViz (使用现有配置)"
echo "4. 测试 RViz 启动"
echo ""

read -p "请选择启动选项 [1-4]: " choice

case $choice in
    1)
        echo "启动 Livox 雷达融合系统 (包含 RViz)..."
        roslaunch fast_lio merge_lidar_livox.launch
        ;;
    2)
        echo "启动多雷达融合系统 (包含 RViz)..."
        roslaunch fast_lio merge_lidar.launch rviz:=true
        ;;
    3)
        echo "仅启动 RViz..."
        rviz -d "$RVIZ_CONFIG"
        ;;
    4)
        echo "测试 RViz 启动 (3秒后自动关闭)..."
        timeout 3s rviz -d "$RVIZ_CONFIG" || echo "RViz 测试完成"
        ;;
    *)
        echo "无效选择，启动默认配置..."
        roslaunch fast_lio merge_lidar_livox.launch
        ;;
esac
