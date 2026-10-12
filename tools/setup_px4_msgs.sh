#!/usr/bin/env bash
#
# 拉取与 PX4 1.17 固件匹配的 px4_msgs 到工作区 src/ 下。
#
# 为什么不用 submodule / 不入库：
#   px4_msgs 是第三方生成包（几百个 .msg 生成出来的 C++ 头），体积大、变更频繁，
#   而且**必须与飞控上烧的固件版本严格对应**。把它放进本仓库会让仓库膨胀，
#   也容易让人误以为"仓库里这份就是对的"。所以走"脚本按固定分支 + 固定 sha 拉取"。
#
# 版本硬约束（见 docs/PX4伴飞工程计划书.md §11）：
#   飞控固件 PX4 v1.17.0  ->  px4_msgs 分支 release/1.17
#   消息定义不一致时，编译能过，但话题字段会错位 —— 这类错误极难排查。
#
# 用法：
#   bash tools/setup_px4_msgs.sh            # 没有就 clone，有就切到目标分支
#   bash tools/setup_px4_msgs.sh --force    # 先删掉再重来
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${REPO_ROOT}/src/px4_msgs"
URL="https://github.com/PX4/px4_msgs.git"
BRANCH="release/1.17"
EXPECTED_SHA="86d8239e962f6939e05c3737784f60c02fa884db"

FORCE=0
if [[ "${1:-}" == "--force" ]]; then
  FORCE=1
fi

echo "== px4_msgs 准备 =="
echo "目标目录 : ${DEST}"
echo "分支     : ${BRANCH}"
echo "期望 sha : ${EXPECTED_SHA}"
echo

if [[ "${FORCE}" == "1" && -d "${DEST}" ]]; then
  echo "--force：删除已存在的 ${DEST}"
  rm -rf "${DEST}"
fi

if [[ -d "${DEST}/.git" ]]; then
  echo "已存在 git 仓库，拉取更新…"
  git -C "${DEST}" fetch --tags origin
  git -C "${DEST}" checkout "${BRANCH}"
  git -C "${DEST}" pull --ff-only origin "${BRANCH}"
else
  if [[ -e "${DEST}" ]]; then
    echo "错误：${DEST} 已存在但不是 git 仓库。确认后加 --force 重来。" >&2
    exit 1
  fi
  echo "克隆…"
  git clone --branch "${BRANCH}" --single-branch "${URL}" "${DEST}"
fi

ACTUAL_SHA="$(git -C "${DEST}" rev-parse HEAD)"
echo
echo "实际 sha : ${ACTUAL_SHA}"
if [[ "${ACTUAL_SHA}" != "${EXPECTED_SHA}" ]]; then
  cat >&2 <<EOF
⚠ 注意：实际 sha 与本项目记录的不同。
  记录值 ${EXPECTED_SHA}
  实际值 ${ACTUAL_SHA}
  这不一定是错误（分支可能已前进），但如果你正在排查"话题字段对不上"这类问题，
  请优先怀疑这里。确认无误后，把新 sha 同步回 docs/PX4伴飞工程计划书.md §11。
EOF
fi

cat <<'EOF'

下一步：
  1) 编译：
       colcon build --symlink-install
  2) 确认消息定义已可用（应当打印出字段列表，而不是报 package not found）：
       source install/setup.bash
       ros2 interface show px4_msgs/msg/VehicleOdometry
       ros2 interface show px4_msgs/msg/OffboardControlMode
       ros2 interface show px4_msgs/msg/TrajectorySetpoint

提示：如果 git clone 卡住或超时，多半是网络问题。树莓派上若走代理，先设置：
  export https_proxy=http://127.0.0.1:7897
  export http_proxy=http://127.0.0.1:7897
EOF
