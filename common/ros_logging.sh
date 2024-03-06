#!/bin/bash

# 源目录和目标目录
source_directory="$HOME/pix/ros-log"
target_directory="$HOME/pix/ros-log/old"

exec 2> $ROS_LOG_DIR/"1_ros_logging_stderr.log"
exec 1> $ROS_LOG_DIR/"1_ros_logging_stdout.log"

# 确保目标目录存在
mkdir -p "$target_directory"

# 获取一周前的日期（以秒为单位）
week_ago=$(date -d "1 week ago" +%s)

# 遍历源目录中的所有文件
for folder in "$source_directory"/*; do
    if [[ -d "$folder" && "$folder" =~ autoware_([0-9]{14}) ]]; then
        # 提取时间戳部分并转换为日期格式
        timestamp="${BASH_REMATCH[1]}"
        folder_date=$(date -d "${timestamp:0:4}-${timestamp:4:2}-${timestamp:6:2} ${timestamp:8:2}:${timestamp:10:2}:${timestamp:12:2}" +%s)
        # 如果文件夹日期早于一周前，则删除该文件夹
        if [ "$folder_date" -lt "$week_ago" ]; then
            echo "一周前日志文件夹: $folder"
            # 移动文件到目标目录
            mv "$folder" "$target_directory/"
        fi
    fi
done

# rm -r "$HOME/pix/ros-log/old/*"

function create_symlink_for_latest_folder() {
    local target_directory=$1
    local prefix=$2
    local latest_timestamp=0
    local latest_folder=""

    # 检查目标目录是否存在
    if [ ! -d "$target_directory" ]; then
        echo "目录不存在: $target_directory"
        return 1
    fi

    # 遍历目标目录中的所有文件夹
    for folder in "$target_directory"/$prefix*; do
        if [[ -d "$folder" && $(basename "$folder") =~ ^$prefix[0-9]{14}$ ]]; then
            # 提取时间戳并与已知的最新时间戳比较
            timestamp=$(basename "$folder" | sed -e "s/^$prefix//")
            if [[ "$timestamp" > "$latest_timestamp" ]]; then
                latest_timestamp=$timestamp
                latest_folder=$folder
            fi
        fi
    done

    # 检查是否找到最新的文件夹
    if [ -z "$latest_folder" ]; then
        echo "未找到符合格式的文件夹: $prefix"
        return 1
    fi

    # 为最新的文件夹创建软链接
    local symlink_name=1_"$prefix"latest
    ln -sfn "$latest_folder" "$target_directory/$symlink_name"
    echo "为最新文件夹 $latest_folder 创建了软链接 $symlink_name"
}

create_symlink_for_latest_folder "$HOME/pix/ros-log" "autoware_"