cmake_minimum_required(VERSION 3.16)

# =============================================================================
#  视觉补盲（方案任务一 + 任务三的视觉侧）
#
#  「已知位姿的结构恢复」：不做 SLAM，用导航给的位姿把特征点三角化 + M-A 局部 BA，
#  结果走三条出口（C3 实时障碍 / C4 建图点云 / C5 状态）交给导航。
#
#  依赖只有 Eigen + OpenCV；ROS2 只通过 io::Publish2NavPointCloud 接入，
#  没有 ROS2 时（SP_WITH_ROS2 未定义）整段退化为空实现，不影响其他目标编译。
# =============================================================================
add_library(visual_depth OBJECT
    # 单相机跨帧特征跟踪（goodFeaturesToTrack + LK + 前后向校验）
    feature_tracker.cpp
    # 固定位姿三角化（DLT + LM）与 M-A 局部 BA、多帧一致性
    triangulator.cpp
    # 地面平面反投影（纯旋转兜底）
    ground_projection.cpp
    # 建图点落盘（出口 C 的离线 PCD）
    pcd_writer.cpp
    # 总装：把 PnP 点与三角化点汇总成 C3/C4/C5
    visual_depth.cpp
)

# io 的 usage requirements（Eigen/OpenCV 由顶层统一 include；ROS2 头与 SP_WITH_ROS2 由 io 传播）
target_link_libraries(visual_depth io)
