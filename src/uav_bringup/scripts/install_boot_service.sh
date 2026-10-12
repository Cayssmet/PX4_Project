#!/usr/bin/env bash
set -euo pipefail

UAV_SERVICE_SOURCE="/home/ljz/ros2_ws/install/uav_bringup/share/uav_bringup/systemd/uav-navigation.service"
UAV_SERVICE_TARGET="/etc/systemd/system/uav-navigation.service"

if [[ ! -f "${UAV_SERVICE_SOURCE}" ]]; then
  echo "未找到 ${UAV_SERVICE_SOURCE}，请先构建 uav_bringup 包"
  exit 1
fi

echo "即将安装并启用 uav-navigation.service；本步骤需要输入本机sudo密码。"
sudo install -m 0644 "${UAV_SERVICE_SOURCE}" "${UAV_SERVICE_TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable uav-navigation.service

echo "服务已设为开机启动，但当前没有立即启动。"
echo "校准和无动力检查完成后，可执行：sudo systemctl start uav-navigation.service"
echo "查看状态：systemctl status uav-navigation.service"
