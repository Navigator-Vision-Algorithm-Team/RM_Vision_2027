# 步兵相机安装偏移

相机与执行轴平行，相机光心在执行轴左侧 77.64 mm、上方约 5 mm。
前后距离尚未提供，当前按 0 m 处理；执行轴原点作为瞄准计算原点。

相机坐标为 x 向右、y 向下、z 向前；执行机构（gimbal）坐标为 x 向前、y 向左、z 向上。
`configs/soldier.yaml` 和 `configs/soldier_virtual.yaml` 使用相同外参：

```yaml
R_camera2gimbal: [0, 0, 1, -1, 0, 0, 0, -1, 0]
t_camera2gimbal: [0.0, 0.07764, 0.005] # m
reticle_distance_m: 3.0
```

PnP 得到相机系位置后，`Solver::solve` 先计算
`p_gimbal = R_camera2gimbal * p_camera + t_camera2gimbal`，再转换到世界系。
Tracker 和 Aimer 使用这个结果，所以跟踪、目标预测、yaw 和弹道高度均已包含安装偏移。
不要再把这两个长度加到 `yaw_offset` / `pitch_offset`；这两个参数仍只用于角度微调。

调试图像 `/sp_debug/<tag>/image` 上的绿色 `AXIS` 十字表示当前执行轴在指定深度处的几何投影。
跟踪入口通过 `debug.publish(..., &solver)` 传入同帧实际使用的 Solver，
使用它的旋转矩阵和外参计算准星，只把投影结果交给异步绘图线程。
因此不需要从调试欧拉角重建姿态，运行时外参更新也会同时反映到准星。
跟踪入口按预测目标在当前 gimbal 系的前向深度更新准星；没有有效预测点时使用
`reticle_distance_m`，默认为配置中的 3 m。`yolodebug` 不计算目标深度，因此使用该参考距离。
画面标注同时显示所用深度。原始图像和 YOLO 输入图不叠加准星。
需启用 `debug_ros2.enable` 才会发布调试画面。
正式 CAN 入口 `soldier` 也接入了此调试流；核对实车时应观察该入口的图像，
以保证准星、目标预测点和 CAN 指令使用同一帧姿态。
`soldier_debug` 仍是串口调试入口，`yolodebug` 则只显示参考深度下的准星。

相机在左上方，因此执行轴投影应位于相机主点的右下方，且距离越近偏移越大。
忽略畸变时，深度为 `d` 米的像素坐标为
`u = cx + fx * 0.07764 / d`、`v = cy + fy * 0.005 / d`。
实际代码通过 OpenCV 投影加入畸变；主点来自相机内参。
绿色准星表示几何轴线，品红色预测点表示目标提前量；弹道下坠由 Aimer 计算。
两者使用相同的安装偏移，但绿色 `AXIS` 不表示弹道落点：有重力下坠时，
即使正确瞄准，预测目标与当前执行轴也可能有垂直间距。

方向与重复补偿的互逆检查：

- 目标在相机正前方 3 m：`p_camera = [0, 0, 3]`，
  应得到 `p_gimbal = [3, 0.07764, 0.005]`，几何 yaw 为向左约 1.48248°，
  几何仰角约 0.09546°；Aimer 的执行 pitch 向上为负，另外再计算重力补偿。
- 目标在执行轴前方 3 m：`p_camera = [0.07764, 0.005, 3]`，
  应得到 `p_gimbal = [3, 0, 0]`，几何 yaw 和高度误差均为 0；
  该点的像素必须与绿色准星重合。
- 求解使用 `R * p_camera + t`；准星反投影使用 `R.transpose() * (p_gimbal - t)`。
  这里的减号是同一坐标变换的逆变换，不是给控制指令再加一次补偿。
- `yaw_offset`、`pitch_offset` 在两份步兵配置中均为 0。
  CAN 的 pitch 零点/方向转换只处理编码器角度，不包含相机安装长度。

无需相机、CAN、ROS2 或 OpenVINO 的几何回归测试（需要 OpenCV、Eigen3、yaml-cpp）：

```bash
cmake -S tests/camera_geometry -B build/camera_geometry
cmake --build build/camera_geometry --parallel
ctest --test-dir build/camera_geometry --output-on-failure
```

本次复核通过两份配置在 1、3、10 m 的合成 PnP 互逆检查，以及非零 yaw/pitch/roll 的
坐标往返检查；最大合成 PnP 位置误差约 0.00373 mm。
另已在 Windows 用 g++ 编译并运行生产 `tools/trajectory.cpp` 的方向测试，三种距离均通过。
完整 OpenCV C++ 回归测试尚未运行；本机缺少相应开发依赖和 Linux 发行版。
完整工程还存在既有缺失文件 `tools/tuning.hpp`，需要补齐后才能做整工程构建验证。
