#!/usr/bin/env bash
set -eo pipefail

UAV_ROS_LOG_DIRECTORY="${ROS_LOG_DIR:-/home/ljz/.ros/log}"
export ROS_LOG_DIR="${UAV_ROS_LOG_DIRECTORY}"
mkdir -p "${UAV_ROS_LOG_DIRECTORY}"

# ROS 2 的 setup.bash 会读取若干可选环境变量，加载期间不能启用 nounset。
source /opt/ros/jazzy/setup.bash
source /home/ljz/ros2_ws/install/setup.bash
set -u

UAV_RADAR_DEVICE="${UAV_LIDAR_SERIAL_DEVICE:-/dev/wheeltec_laser}"
UAV_FLIGHT_DEVICE="${UAV_FLIGHT_SERIAL_DEVICE:-/dev/flight_controller}"
UAV_FLIGHT_PROTOCOL="${UAV_FLIGHT_PROTOCOL_VERSION:-2}"
UAV_CAMERA_DEVICE="${UAV_CAMERA_SERIAL_DEVICE:-/dev/ttyAMA0}"
UAV_CAR_DEVICE="${UAV_CAR_SERIAL_DEVICE:-/dev/car_bluetooth}"
UAV_GCS_DEVICE="${UAV_GCS_SERIAL_DEVICE:-/dev/gcs_bluetooth}"
UAV_COMPETITION_TEST_START="${UAV_COMPETITION_AUTO_START_FOR_TEST:-false}"
UAV_RIGHT_SHIFT_LAND_TEST="${UAV_COMPETITION_RIGHT_SHIFT_LAND_TEST:-false}"
UAV_RVIZ_SETTING="${UAV_USE_RVIZ:-false}"

for UAV_WAIT_ATTEMPT in $(seq 1 30); do
  if [[ -e "${UAV_RADAR_DEVICE}" ]]; then
    break
  fi
  echo "等待雷达设备 ${UAV_RADAR_DEVICE}（${UAV_WAIT_ATTEMPT}/30）"
  sleep 1
done

if [[ ! -e "${UAV_RADAR_DEVICE}" ]]; then
  echo "雷达设备 ${UAV_RADAR_DEVICE} 在30秒内未出现，退出并等待systemd重启"
  exit 1
fi

if [[ ! -e "${UAV_FLIGHT_DEVICE}" ]]; then
  echo "警告：飞控设备 ${UAV_FLIGHT_DEVICE} 尚未出现；串口桥将保持安全零输出并自动重连"
fi

if [[ ! -e "${UAV_CAR_DEVICE}" ]]; then
  echo "警告：小车设备 ${UAV_CAR_DEVICE} 尚未出现；小车桥将安全等待并自动重连"
fi

if [[ ! -e "${UAV_GCS_DEVICE}" ]]; then
  echo "警告：地面站设备 ${UAV_GCS_DEVICE} 尚未出现；地面站桥将等待重连，飞行任务继续启动"
fi

exec ros2 launch uav_bringup full_system.launch.py \
  use_rviz:="${UAV_RVIZ_SETTING}" \
  competition_auto_start_for_test:="${UAV_COMPETITION_TEST_START}" \
  competition_right_shift_land_test:="${UAV_RIGHT_SHIFT_LAND_TEST}" \
  flight_serial_device:="${UAV_FLIGHT_DEVICE}" \
  flight_protocol_version:="${UAV_FLIGHT_PROTOCOL}" \
  camera_serial_device:="${UAV_CAMERA_DEVICE}" \
  car_serial_device:="${UAV_CAR_DEVICE}" \
  gcs_serial_device:="${UAV_GCS_DEVICE}" \
  lidar_serial_device:="${UAV_RADAR_DEVICE}"
