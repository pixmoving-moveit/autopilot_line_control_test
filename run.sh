#!/bin/bash
source ~/.bashrc
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

function log_info() {
  echo -e "\033[32m[INFO] $*\033[0m[\033[32m\u2714\033[0m]"
}

function log_warning() {
  echo -e "\033[33m[WARNING] $*\033[0m[\033[33m⚠️\033[0m]"
}

function log_error() {
  echo -e "\033[31m[ERROR] $*\033[0m[\033[31m\xE2\x9C\x97\033[0m]"
}

# ------------------------------------------------------------------------
# 设置日志文件报保存路径
timestamp=$(date +"%Y%m%d%H%M%S")
mkdir $HOME/pix/ros-log/"speed_steering_brake_test_$timestamp"
export ROS_LOG_DIR=$HOME/pix/ros-log/"speed_steering_brake_test_$timestamp"

log_info "删除一周前日志"
gnome-terminal -t "remove_log"  -- bash -i -c "$SCRIPT_DIR/common/ros_logging.sh"
# ------------------------------------------------------------------------

# 启动测试程序
source ~/pix/robobus/autoware-robobus/install/setup.bash

exec 2> $ROS_LOG_DIR/"1_stderr.log"
exec 1> $ROS_LOG_DIR/"1_stdout.log"

ros2 launch "$SCRIPT_DIR/common/test.launch.xml" script_path:=$SCRIPT_DIR