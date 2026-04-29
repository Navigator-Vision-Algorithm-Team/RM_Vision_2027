#!/bin/bash

# 设置环境变量
source /home/ubuntu/ros_ws/install/setup.bash

# 定义清理函数
cleanup() {
  echo "清理进程..."
  [[ -n $FOXGLOVE_PID ]] && kill $FOXGLOVE_PID 2>/dev/null
  [[ -n $VISION_PID ]] && kill $VISION_PID 2>/dev/null
   [[ -n $NAV_PID ]] && kill $NAV_PID 2>/dev/null
  exit 1
}

# 捕获中断信号
trap cleanup SIGINT SIGTERM

# 启动Foxglove桥接
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 & 
FOXGLOVE_PID=$!

# 启动视觉系统
ros2 launch rm_vision_bringup vision_bringup.launch.py &
#ros2 launch rm_serial_driver serial_driver.launch.py &
VISION_PID=$!

# 切换到导航环境
source /home/ubuntu/navigation_2025/ros_ws/install/setup.bash

# 启动导航
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=final2 slam:=False use_robot_state_pub:=True &
NAV_PID=$!

echo "所有进程已启动: Vision($VISION_PID), Navigation($NAV_PID)"

# 监控进程，任何一个退出就终止所有进程
while true; do
  #if ! kill -0 $FOXGLOVE_PID 2>/dev/null; then
  #  echo "Foxglove进程已退出,终止所有进程"
  #  cleanup
  #fi
  
  if ! kill -0 $VISION_PID 2>/dev/null; then
    echo "视觉系统进程已退出,终止所有进程"
    cleanup
  fi
  
  if ! kill -0 $NAV_PID 2>/dev/null; then
    echo "导航进程已退出,终止所有进程"
    cleanup
  fi
  
  sleep 1
done
