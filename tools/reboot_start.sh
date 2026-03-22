#!/bin/bash

# the key
# @reboot /path/to/your/script.sh
# ros2 run nav2_map_server map_saver_cli -f dongda --ros-args -r __ns:=/red_standard_robot1
#
# ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x:2.0, y:3.0, z:0.0}, orientation: {x:0.0, y:0.0, z:0.0, w:1.0}}}"
#
# ros2 action send_goal --feedback /red_standard_robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 2.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
#
# ros2 launch rm_vision_bringup vision_bringup.launch.py
#
#
# 编译
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#
#
# source install/setup.bash
#
# 爱来自电控
# source ~/ros_ws/install/setup.bash
#
# 启动自瞄
# ros2 launch rm_vision_bringup vision_bringup.launch.py
#
# 编译
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash && ros2 launch rm_vision_bringup vision_bringup.launch.py
#
# 自瞄可视化
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
#
# 启动建图
# ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
# slam:=True \
# use_robot_state_pub:=True
#
# 导航模式
# ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=307_2026_1 slam:=False use_robot_state_pub:=True
# 导航模式2 新建图
# ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=dona slam:=False use_robot_state_pub:=True
# 导航模式3 新建图1s1
# ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=dong slam:=False use_robot_state_pub:=True
# 单独启动串口
# ros2 launch rm_serial_driver serial_driver.launch.py
#!/bin/bash

# 设置环境变量

source /home/ubuntu/Freshman/RM_Vision_2026/install/setup.bash

# 定义清理函数
cleanup() {
  echo "清理进程..."
  [[ -n $FOXGLOVE_PID ]] && kill $FOXGLOVE_PID 2>/dev/null
  [[ -n $VISION_PID ]] && kill $VISION_PID 2>/dev/null
  [[ -n $NAV_PID ]] && kill $NAV_PID 2>/dev/null
  exit 1
}

trap cleanup SIGINT SIGTERM

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
FOXGLOVE_PID=$!

# ros2 launch rm_vision_bringup vision_bringup.launch.py &
# #ros2 launch rm_serial_driver serial_driver.launch.py &
# VISION_PID=$!

# should we open the rm_serial_driver topic?
ros2 launch rm_vision_bringup vision_bringup.launch.py &
VISION_PID=$!

source /home/ubuntu/navigation_2025/ros_ws/install/setup.bash

source /home/ubuntu/ros_ws/install/setup.bash

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True &
NEV_PID=$!

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

# # 定义清理函数
# cleanup() {
#   echo "清理进程..."
#   [[ -n $FOXGLOVE_PID ]] && kill $FOXGLOVE_PID 2>/dev/null
#   [[ -n $VISION_PID ]] && kill $VISION_PID 2>/dev/null
#    [[ -n $NAV_PID ]] && kill $NAV_PID 2>/dev/null
#   exit 1
# }
#
# # 捕获中断信号
# trap cleanup SIGINT SIGTERM
#
# # 启动Foxglove桥接
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
# FOXGLOVE_PID=$!
#
# # 启动视觉系统
# ros2 launch rm_vision_bringup vision_bringup.launch.py &
# #ros2 launch rm_serial_driver serial_driver.launch.py &
# VISION_PID=$!
#
# # 切换到导航环境
# source /home/ubuntu/navigation_2025/ros_ws/install/setup.bash
#
# # 启动导航
# ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=rmul_2025 slam:=False use_robot_state_pub:=True &
# NAV_PID=$!
#
# echo "所有进程已启动: Vision($VISION_PID), Navigation($NAV_PID)"
#
# # 监控进程，任何一个退出就终止所有进程
# while true; do
#   #if ! kill -0 $FOXGLOVE_PID 2>/dev/null; then
#   #  echo "Foxglove进程已退出,终止所有进程"
#   #  cleanup
#   #fi
#
#   if ! kill -0 $VISION_PID 2>/dev/null; then
#     echo "视觉系统进程已退出,终止所有进程"
#     cleanup
#   fi
#
#   if ! kill -0 $NAV_PID 2>/dev/null; then
#     echo "导航进程已退出,终止所有进程"
#     cleanup
#   fi
#
#   sleep 1
# done
