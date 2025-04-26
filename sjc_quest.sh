#!/bin/bash

# 默认端口（可通过命令行参数传入）
PORT=${1:-8012}

echo "📌 正在准备连接 Oculus Quest 并设置端口转发到端口 $PORT"

# Step 0: 检查 adb 是否安装
if ! command -v adb &> /dev/null; then
    echo "⚠️  未检测到 adb，尝试自动安装..."

    if [ -x "$(command -v apt)" ]; then
        sudo apt update && sudo apt install -y adb
    else
        echo "❌ 当前系统不支持自动安装 adb。请手动安装后再运行脚本。"
        exit 1
    fi
fi

# Step 1: 重启 adb 服务
echo "🔌 Step 1: 尝试重启 adb 服务..."
adb kill-server
adb start-server

# Step 2: 检查是否通过 USB 授权连接
echo "🧩 Step 2: 检查 USB 是否连接 Quest..."
sleep 2
DEVICE_LIST=$(adb devices | grep -w "device" | awk '{print $1}')

if [ -z "$DEVICE_LIST" ]; then
  echo "❌ 没检测到已授权的设备，请通过 USB 连接 Quest 并授权！"
  exit 1
fi

echo "✅ 检测到设备: $DEVICE_LIST"

# Step 3: 获取 Quest IP
echo "🌐 Step 3: 获取 Quest 的 IP 地址..."
IP_LINE=$(adb shell ip route | grep 'wlan0')
QUEST_IP=$(echo $IP_LINE | awk '{print $9}')

if [ -z "$QUEST_IP" ]; then
  echo "❌ 未能获取 Quest 的 IP 地址。请确认设备已连接 Wi-Fi。"
  exit 1
fi

echo "📶 Quest IP: $QUEST_IP"

# Step 4: 启用 tcpip 模式
echo "🔄 Step 4: 启用 ADB 无线调试模式..."
adb tcpip 5555
sleep 2

# Step 5: 无线连接 Quest
echo "📡 Step 5: 通过 IP ($QUEST_IP:5555) 尝试无线连接 Quest..."
adb connect $QUEST_IP:5555

# Step 6: 端口反向转发
echo "🔁 Step 6: 设置 adb reverse 端口转发: $PORT"

# 查找 Quest 的无线设备 ID（IP:PORT 格式）
WIRELESS_ID=$(adb devices | grep "$QUEST_IP:5555" | awk '{print $1}')

if [ -z "$WIRELESS_ID" ]; then
  echo "❌ 未找到无线设备（$QUEST_IP:5555），请确认连接成功。"
  exit 1
fi

# 对指定设备执行 adb reverse
adb -s "$WIRELESS_ID" reverse tcp:$PORT tcp:$PORT
# 提示
echo ""
echo "✅ 成功完成无线连接和端口转发！你现在可以在 Quest 的 Oculus 浏览器中访问："
echo "   http://localhost:$PORT"
echo ""
echo "📎 如果你断开了 Quest，下次运行此脚本即可重新连接。"
