#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/serial/serial_board.hpp"
#include "io/ros2/ros2.hpp"
#include "io/tf/tf_tree.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/yolo_pipeline.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/omniperception/decider.hpp"
// 【改动】load_omni_pnp_config（读取全向相机 PnP 标定参数）声明在这里
#include "tasks/omniperception/omni_jobs.hpp"
#include "tasks/omniperception/omni_pnp.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Plotter plotter;
  tools::Recorder main_recorder(30, "main");

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  auto yaml = tools::load(config_path);

  // 坐标变换树：所有几何量（每个相机相对旋转轴的位置/姿态）都来自配置 tf.frames，
  // 代码里不写死任何安装尺寸。大yaw 与小yaw 作为两个动态关节，每帧由 0x502 刷新。
  io::tf::Tree tf_tree = io::tf::load_tree(yaml);
  if (tf_tree.empty()) {
    tools::logger()->warn(
      "[Main] 配置里没有 tf.frames：主相机/全向相机将使用各自的回退外参（未标定则为安装角近似）");
  } else {
    std::string frames;
    for (const auto & f : tf_tree.frames()) frames += f + " ";
    tools::logger()->info("[Main] tf frames: {}", frames);
  }

  tools::logger()->info("[Main] Initializing SerialBoard...");
  // 串口初始化
  io::SerialBoard serial_board(config_path);
  tools::logger()->info("[Main] SerialBoard ready.");

  // 启动时先打一次比赛信息：此刻裁判帧通常还没到，重点是让现场确认
  // 「裁判系统是否启用、robot_id 有没有上报、弹速是多少」。
  io::log_match_info(serial_board, "启动");

  // ROS2 初始化：内部启动独立线程，收到导航数据后立即经串口下发，
  // 与主循环（视觉处理）解耦，避免被主链路延迟。
  io::ROS2 ros2([&serial_board](const io::NavCommand & nav_command) {
    serial_board.send(nav_command);
  });

  // 云台关节反馈：复用 SerialBoard 已有的串口接收线程，收到一帧 0x502 就发布一次
  // JointState，不额外启动程序、不重复占用串口。
  serial_board.set_gimbal_joint_callback(ros2.gimbal_joint_sender());

  // 裁判状态转发：导航决策节点（pb_nav_goal_decision）在 red_standard_robot1 命名空间下
  // 订阅 referee/{game_progress,current_hp,maximum_hp}，数据源是串口 0x0001 / 0x0201 帧。
  // 未接裁判系统（use_referee_system=false）且下位机不转发 0x0001 时收不到数据，这里什么都
  // 不发布，导航侧停在 WAIT_GAME_START 不动作 —— 刻意保留，避免无裁判数据时自走。
  // 配置 test_nav: true 时 referee_state() 直接返回「比赛已开始 + 默认血量」，与下位机无关。
  ros2.set_referee_source([&serial_board]() { return serial_board.referee_state(); });

  // Exiter 必须在 io::ROS2 之后构造：rclcpp::init 会用自己的 handler 接管 SIGINT，
  // 后安装的 handler 才能让 Ctrl+C 重新置位退出标志。
  tools::Exiter exiter;

  tools::logger()->info("[Main] Opening camera...");
  // 相机初始化
  io::Camera camera(config_path, "main");
  tools::logger()->info("[Main] Camera ready: {}", camera.device_name());

  int omni_count = 0;
  if (yaml["omni_camera_count"]) omni_count = yaml["omni_camera_count"].as<int>();

  // 全局波形关闭时，单主相机丢失目标后的旧行为开关。
  const bool spin_when_no_omni =
    yaml["spin_when_no_omni"] ? yaml["spin_when_no_omni"].as<bool>() : true;
  const bool gimbal_wave_motion =
    yaml["gimbal_wave_motion"] ? yaml["gimbal_wave_motion"].as<bool>() : true;
  if (omni_count == 0) {
    tools::logger()->info(
      "[Main] legacy spin when lost: {}", spin_when_no_omni ? "on" : "off");
  }
  tools::logger()->info(
    "[Main] gimbal wave motion: {}", gimbal_wave_motion ? "on (shoot disabled)" : "off");

  // 缓冲时间， 确保全向相机初始化完成，避免出现空帧，但是我们的全向相机和主相机并不在同一个物理链路上，所以这个sleep并没有非常的必要
  // if (omni_count > 0) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // }

  // 初始化全向相机
  std::vector<std::unique_ptr<io::Camera>> omni_cameras;
  std::vector<omniperception::OmniCameraConfig> omni_configs;

  for (int i = 1; i <= omni_count; i++) {
    auto sursign = "omni" + std::to_string(i);
    auto cam = std::make_unique<io::Camera>(config_path, sursign);
    auto recorder = std::make_unique<tools::Recorder>(30, sursign);

    //缓冲时间
    if (i < omni_count) {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    //配置设置
    omniperception::OmniCameraConfig cfg;
    cfg.camera = cam.get();
    // 图像方向修正：io::Camera 已按安装方位把图转了 90°，
    // 这里把同一角度带给 PnP，用于把标定内参变换到旋转后的坐标系。
    cfg.rotate = cam->rotation();
    cfg.recorder = std::shared_ptr<tools::Recorder>(std::move(recorder));
    cfg.mount_yaw = yaml["omni_mount_yaw_" + std::to_string(i)]
                      ? yaml["omni_mount_yaw_" + std::to_string(i)].as<double>()
                      : 0.0;
    cfg.mount_pitch = yaml["omni_mount_pitch_" + std::to_string(i)]
                        ? yaml["omni_mount_pitch_" + std::to_string(i)].as<double>()
                        : 0.0;
    cfg.fov_h = yaml["omni_fov_h_" + std::to_string(i)]
                  ? yaml["omni_fov_h_" + std::to_string(i)].as<double>()
                  : 54.2;
    cfg.fov_v = yaml["omni_fov_v_" + std::to_string(i)]
                  ? yaml["omni_fov_v_" + std::to_string(i)].as<double>()
                  : 44.5;

    // =====================================================================
    //  读取该全向相机的 PnP 标定参数（内参/畸变/外参，全部可选）。
    //  键名： omni_camera_matrix_N / omni_distort_coeffs_N /
    //         omni_R_camera2gimbal_N / omni_t_camera2gimbal_N
    //  解析实现见 tasks/omniperception/omni_pnp.cpp::load_omni_pnp_config。
    //  注意：该函数只做解析，"外参键缺失"时【不】报警——因为那一刻还不知道
    //        tf 树会不会兜底，报警会与实际行为矛盾。日志由下面这段按三态统一输出。
    // =====================================================================
    omniperception::load_omni_pnp_config(cfg, yaml, i);

    // 外参优先级：tf 树 > yaml 的 omni_R/t_camera2gimbal_N > 安装角近似。
    // 三态日志（避免"明明用了 tf 却报警说要用安装角"的误导）：
    //   ① tf 查到          -> debug：实车正常路径，不打扰
    //   ② tf 无、yaml 有    -> info ：合法降级（例如仿真配置没写 tf）
    //   ③ tf 无、yaml 也无  -> warn ：真的在走安装角近似（t=0，近距离有视差误差）
    // 进入本块时 cfg.has_extrinsic 的含义是「yaml 是否给了有效外参」。
    {
      io::tf::Transform T_cam2yaw;
      const std::string frame = io::tf::Tree::OMNI_CAM_PREFIX + std::to_string(i);

      if (tf_tree.lookup(frame, io::tf::Tree::YAW_AXIS, T_cam2yaw)) {
        // ① tf 优先：omni_cam_i -> yaw_axis 给出「相机 -> 大yaw」的完整位姿（含平移）。
        //    平移很关键：全向相机的光心不在大yaw 转轴上，只用旋转会带来视差误差。
        cfg.R_camera2gimbal = T_cam2yaw.R;
        cfg.t_camera2gimbal = T_cam2yaw.t;
        cfg.has_extrinsic = true;
        tools::logger()->debug(
          "[omni{}] 外参 <- tf ({} -> {})", i, frame, io::tf::Tree::YAW_AXIS);
      } else if (!cfg.has_extrinsic) {
        // ③ 两个来源都没有：decider 会退回 mount_to_R_camera2gimbal()，且 t 置 0。
        tools::logger()->warn(
          "[omni{}] tf 里没有 {} -> {}，且未配置 omni_R/t_camera2gimbal_{} "
          "-> 将用安装角 mount_yaw/mount_pitch 近似外参（t=0，近距离有视差误差）",
          i, frame, io::tf::Tree::YAW_AXIS, i);
      } else {
        // ② tf 缺失但 yaml 给了有效 R/t：合法降级，不是错误。
        tools::logger()->info(
          "[omni{}] tf 里没有 {} -> {}，外参沿用 yaml 的 omni_R/t_camera2gimbal_{}",
          i, frame, io::tf::Tree::YAW_AXIS, i);
      }
    }

    omni_configs.push_back(cfg);
    omni_cameras.push_back(std::move(cam));
  }

  // 初始化其他组件
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);

  // 配置里的 enemy_color 只是「还没收到 0x0502 roboid」时的兜底。启动时先把它
  // 推断出的己方/敌方颜色打出来，现场一眼就能确认视觉认为自己是哪一方。
  const auto config_enemy_color =
    yaml["enemy_color"].as<std::string>() == "red" ? auto_aim::Color::red
                                                  : auto_aim::Color::blue;
  const auto config_own_color =
    config_enemy_color == auto_aim::Color::red ? auto_aim::Color::blue : auto_aim::Color::red;
  tools::logger()->info(
    "[Main] 己方颜色（配置兜底）= {}，敌方 = {}；收到 0x0502 roboid 后按编号表覆盖",
    auto_aim::COLORS[static_cast<int>(config_own_color)],
    auto_aim::COLORS[static_cast<int>(config_enemy_color)]);

  // 全向推理线程与主线程共享的运行时状态（目前只有敌方颜色）。
  // make_omni_jobs 给每路全向建一个独立 Decider，主线程的
  // Decider::set_enemy_color 传不进推理线程；不同步的后果是全向检测到的装甲板
  // 被 armor_filter 按旧颜色全部剔除，表现为「全向感知完全失效」。
  auto omni_state = std::make_shared<omniperception::OmniSharedState>();
  omni_state->set_enemy_color(config_enemy_color);

  // 多相机推理池：每个相机一个独立 YOLO 实例 + 一个绑定固定 CPU 核的线程，
  // 相机数据只喂给自己的实例，不再出现多路相机争抢同一个 OpenVINO InferRequest。
  // 结果进入统一队列，主相机结果始终优先。
  // camera_jobs 里面初始的架构就是这个相机对于yolo的特殊处理配置，像是recoder和yolo推理之后的如何处理之类的事情
  std::vector<auto_aim::multithread::CameraJob> camera_jobs;
  {
    auto_aim::multithread::CameraJob main_job;
    main_job.camera = &camera;
    main_job.camera_id = -1;  // -1 = 主相机
    main_job.keep_image = true;
    main_job.pin_cpu = false;
    main_job.single_thread_cpu = false;
    camera_jobs.push_back(std::move(main_job));

    auto omni_jobs = omniperception::make_omni_jobs(omni_configs, config_path, omni_state);
    for (auto & job : omni_jobs) camera_jobs.push_back(std::move(job));
  }
  auto_aim::multithread::YoloPipeline pipeline(config_path, std::move(camera_jobs), false);

  // 主相机外参：tf 树里有 main_cam -> small_yaw 就用标定值，否则沿用 yaml 里的
  // R_camera2gimbal / t_camera2gimbal（语义同样是「相机 -> 小yaw 板」）。
  {
    io::tf::Transform T_cam2plate;
    if (tf_tree.lookup(io::tf::Tree::MAIN_CAM, io::tf::Tree::SMALL_YAW, T_cam2plate)) {
      solver.set_extrinsic(T_cam2plate.R, T_cam2plate.t);
      tools::logger()->info("[Main] main_cam extrinsic <- tf tree");
    }
  }

  bool game_started_logged = false;

  // 敌我颜色动态识别：串口 0x0502 的 roboid（唯一来源）给出己方 ID，
  // 按附录二编号表（1~11 红方、101~111 蓝方）判定己方颜色，敌方恒为反色。
  // 配置里的 enemy_color 只在还没收到 ID 时兜底，收到后按实际对阵覆盖。
  uint8_t last_robot_id = 0;
  auto last_id_report = std::chrono::steady_clock::now();

    // 全向感知缓存：全向发现目标后的 yaw/pitch，主相机锁定前反复播报
  bool has_omni_cache = false;
  double cached_omni_yaw = 0;
  double cached_omni_pitch = 0;
  bool gimbal_wave_initialized = false;
  std::chrono::steady_clock::time_point gimbal_wave_start;
  double gimbal_wave_yaw_center = 0;
  double gimbal_wave_pitch_center = 0;

  // 主相机最后出帧的时间：用于相机掉线时的告警（超时等待本身不会卡死主循环）
  auto last_main_frame = std::chrono::steady_clock::now();
  auto last_stall_log = last_main_frame;

  // 比赛信息只在状态翻转时打印一次，避免主循环每帧刷屏
  bool last_game_started = serial_board.is_game_started();
  bool last_referee_ready = false;

  // 裁判断流诊断：启用裁判但下位机没转发 0x0001 时，game_started_ 永远是 false，
  // 推理总开关会一直关着，而除此之外没有任何报错。
  auto last_referee_check = std::chrono::steady_clock::now();
  uint64_t last_referee_frames = serial_board.game_status_frame_count();
  bool referee_timeout_warned = false;

  // rclcpp::init 还接管了 SIGTERM（只置 rclcpp 的 shutdown 标志），
  // 因此除 Exiter 的 SIGINT 外再检查 rclcpp::ok()，保证 Ctrl+C / kill 都能退出。
  while (!exiter.exit() && rclcpp::ok()) {
    // 每帧检查一次己方 ID：一旦下位机上报（比赛开始前后都可能），立刻把敌方颜色
    // 换成实际对阵的反色；ID 不变时什么都不做，不会反复清目标。
    if (const uint8_t id = serial_board.robot_id(); id != 0 && id != last_robot_id) {
      last_robot_id = id;
      if (io::is_red_robot_id(id) || io::is_blue_robot_id(id)) {
        const bool own_red = io::is_red_robot_id(id);
        const auto enemy = own_red ? auto_aim::Color::blue : auto_aim::Color::red;
        tracker.set_enemy_color(enemy);
        decider.set_enemy_color(enemy);
        // 全向四路的 Decider 在各自推理线程里，必须经共享状态同步，
        // 否则换色后全向仍按旧颜色过滤，检测结果会被全部剔除。
        omni_state->set_enemy_color(enemy);
        // robot_id 是「颜色是否可信」的唯一依据：没收到之前，全向门控会红蓝都查，
        // 避免配置里的 enemy_color 与实际阵营相反时开赛初期全向完全失效。
        omni_state->set_robot_id(id);
        tools::logger()->info(
          "[Main] robot_id={}（{} / {}）-> 己方颜色 = {}，敌方 = {}（覆盖配置 enemy_color）", id,
          own_red ? "red" : "blue", io::robot_role_name(id), own_red ? "red" : "blue",
          enemy == auto_aim::Color::red ? "red" : "blue");
      } else {
        tools::logger()->warn(
          "[Main] robot_id={} 不在附录二编号表内（1~11 红 / 101~111 蓝），保持配置的 enemy_color",
          id);
      }
    }

    // 低频诊断：还没拿到有效 ID 时，把「0x0502 帧没到」和「帧到了但 roboid=0」
    // 区分开。两种情况都会让颜色停在配置兜底值上，但修法完全不同：
    //   frames == 0  -> 固件包长/CRC 问题，或根本没发 0x0502
    //   raw == 0     -> 帧正常，固件没把 robot_id 填进 roboid
    if (serial_board.robot_id() == 0) {
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - last_id_report).count() >= 5.0) {
        last_id_report = now;
        const auto frames = serial_board.imu_frame_count();
        if (frames == 0) {
          tools::logger()->warn(
            "[Main] 仍未收到 0x0502 帧，无法判定阵营；己方颜色按配置兜底 = {}。"
            "检查下位机 0x0502 是否已带 roboid（包长 26 字节）",
            auto_aim::COLORS[static_cast<int>(config_own_color)]);
        } else {
          tools::logger()->warn(
            "[Main] 0x0502 已收 {} 帧，但 roboid={}（0 = 下位机未填）；"
            "己方颜色按配置兜底 = {}。请确认固件把 robot_id 写进 0x0502 的 roboid",
            frames, serial_board.last_raw_roboid(),
            auto_aim::COLORS[static_cast<int>(config_own_color)]);
        }
      }
    }

    // 开赛前关闭推理：相机仍持续出图（主相机帧照常入队，供录像使用），
    // 避免赛前 5 路 YOLO 空转占满 CPU。
    const bool game_started = serial_board.is_game_started();
    if (game_started != last_game_started) {
      last_game_started = game_started;
      io::log_match_info(serial_board, game_started ? "比赛开始" : "比赛结束");
    }
    pipeline.set_inference_enabled(game_started);

    // 裁判帧首次到齐时补打一次：启动瞬间 0x0001/0x0201 往往还没到，
    // 只打「尚未收到」的话现场看不到真实血量与剩余时间。
    const auto referee = serial_board.referee_state();
    const bool referee_ready = referee.game_status_valid && referee.robot_status_valid;
    if (referee_ready && !last_referee_ready) io::log_match_info(serial_board, "裁判就绪");
    last_referee_ready = referee_ready;

    // 每 5 s 检查一次 0x0001 的帧计数：5 s 不增长说明下位机根本没转发裁判帧，
    // game_started_ 会一直是 false，主相机和全向都不会推理。这条日志是排查
    // 「裁判数据读不到 / 推理不启动」最直接的线索。
    if (serial_board.referee_enabled()) {
      const auto check_now = std::chrono::steady_clock::now();
      if (check_now - last_referee_check >= std::chrono::seconds(5)) {
        const uint64_t frames = serial_board.game_status_frame_count();
        if (frames == last_referee_frames) {
          if (!referee_timeout_warned) {
            referee_timeout_warned = true;
            tools::logger()->warn(
              "[Main] 已启用裁判系统（use_referee_system=true）但 5s 内没有收到 "
              "0x0001/0x0100：is_game_started() 将保持 false，主相机与全向都不会推理。"
              "请检查下位机是否转发裁判帧 / 波特率，或把 use_referee_system 改为 false");
          }
        } else {
          referee_timeout_warned = false;
        }
        last_referee_frames = frames;
        last_referee_check = check_now;
      }
    }

    // 限时等待主相机结果：统一结果队列里主相机的优先级最高。
    // 必须带超时，相机掉线时也能回到 while 条件响应 Ctrl+C / SIGTERM。
    auto_aim::multithread::CameraResult frame;
    const auto status = pipeline.wait_main(frame, std::chrono::milliseconds(100));
    if (status == auto_aim::multithread::WaitStatus::Stopped) break;  // 推理池已停止

    if (status == auto_aim::multithread::WaitStatus::Timeout) {
      const auto now = std::chrono::steady_clock::now();
      const double stalled = std::chrono::duration<double>(now - last_main_frame).count();
      if (stalled >= 2.0 &&
          std::chrono::duration<double>(now - last_stall_log).count() >= 2.0) {
        last_stall_log = now;
        tools::logger()->warn("[Main] 主相机已 {:.1f}s 没有新帧，继续等待...", stalled);
      }
      continue;
    }

    last_main_frame = std::chrono::steady_clock::now();
    if (frame.img.empty()) {
      continue;
    }

    cv::Mat & img = frame.img;
    const auto timestamp = frame.timestamp;
    auto armors = std::move(frame.armors);

    // 双yaw 姿态：大yaw 只有 yaw，pitch/roll 属于小yaw（主相机所在板）
    const io::ImuState imu = serial_board.imu_state_at(timestamp - 1ms);

    // 刷新 tf 树的两个动态关节：
    //   yaw_axis  = Rz(lower_yaw)
    //   small_yaw = Rz(upper)·Ry(pitch)·Rx(roll)
    tf_tree.set_imu(imu.pitch, imu.roll, imu.upper_yaw, imu.lower_yaw);

    // 主相机基座（小yaw 板）姿态：从 tf 树取，代码里不再手工复合欧拉角。
    // tf 里没配 small_yaw 时退回手工复合，保证未标定也能跑。
    Eigen::Quaterniond q_main;
    {
      io::tf::Transform T_plate2world;
      if (tf_tree.lookup(io::tf::Tree::SMALL_YAW, io::tf::Tree::WORLD, T_plate2world)) {
        q_main = Eigen::Quaterniond(T_plate2world.R);
      } else {
        q_main = Eigen::Quaterniond(
          Eigen::AngleAxisd(imu.upper_yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(imu.pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(imu.roll, Eigen::Vector3d::UnitX()));
      }
    }

    // 大yaw（全向相机所在轴）的世界系 yaw：同样从 tf 树取
    double omni_base_yaw = imu.lower_yaw;
    {
      io::tf::Transform T_yaw2world;
      if (tf_tree.lookup(io::tf::Tree::YAW_AXIS, io::tf::Tree::WORLD, T_yaw2world)) {
        omni_base_yaw = std::atan2(T_yaw2world.R(1, 0), T_yaw2world.R(0, 0));
      }
    }

    // 比赛未开始时跳过自瞄逻辑（裁判系统控制）
    if (!serial_board.is_game_started()) {
      if (!game_started_logged) {
        game_started_logged = true;
        tools::logger()->info("[Main] Game not started yet, waiting...");
      }
      //注意这个地方的功耗···检查之后似乎没有很多
      main_recorder.record(img, q_main, timestamp);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (!game_started_logged) {
      game_started_logged = true;
      tools::logger()->info("[Main] Game started! Entering detection & tracking loop");
    }

    // 主相机在小yaw 上，Solver 的 "gimbal" 就是小yaw 板
    //（R_gimbal2imubody 在当前配置里为单位阵，即 tf 的 small_yaw 与 IMU 参考系重合）
    solver.set_R_gimbal2world(q_main);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    if (serial_board.reset_pending()) {
      tracker.reset();
    }

    io::Command command{false, false, 0, 0};

    auto apply_gimbal_wave = [&]() {
      if (!gimbal_wave_initialized) {
        gimbal_wave_initialized = true;
        gimbal_wave_start = std::chrono::steady_clock::now();
        gimbal_wave_yaw_center = gimbal_pos[0];
        gimbal_wave_pitch_center = gimbal_pos[1];
      }

      constexpr double kSpinYawSpeed = 0.5;
      constexpr double kPitchWaveAmplitude = 0.08726646259971647;  // 5 degrees
      constexpr double kPitchWavePeriod = 2.0;
      constexpr double kPi = 3.14159265358979323846;
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - gimbal_wave_start).count();

      command.control = true;
      command.shoot = false;
      command.yaw = tools::limit_rad(gimbal_wave_yaw_center + kSpinYawSpeed * elapsed);
      command.pitch = tools::limit_rad(
        gimbal_wave_pitch_center +
        kPitchWaveAmplitude * std::sin(2.0 * kPi * elapsed / kPitchWavePeriod));
    };

    // 0x0402 报文里没有 control 位（见 io/serial/gimbal/packet.hpp），固件对
    // pitch / yaw 无条件执行，control 只参与 shoot 的与运算。所以「视觉不接管」
    // 也必须下发一个合法角度：这里统一改写成当前姿态，保持云台不动。
    // 少任何一个分支漏填，都会把默认的 0 当成「转到 pitch=0 / 世界系 yaw=0」。
    auto hold_current_pose = [&]() {
      if (command.control) return;
      command.yaw = gimbal_pos[0];
      command.pitch = gimbal_pos[1];
    };

    // 全局波形关闭时，无全向相机且主相机丢失目标按 spin_when_no_omni 扫描或保持。
    // 注意不能直接落到 else 分支：那里在无目标时 tracker/decider 会返回 {false,false,0,0}，
    // 而 0x0402 的 yaw/pitch 是无条件下发的（control 只影响 shoot），
    // 直接走 else 会把云台拽回世界系 0 度。
    if (omni_count == 0 && armors.empty() && tracker.state() == "lost") {
      command.shoot = false;
      if (spin_when_no_omni) {
        // 自旋扫描：云台自旋的角度增量（不是角速度），实际转速 ≈ 该增量 × 帧率
        const double spin_yaw_step = 0.2;
        command.control = true;
        // gimbal_pos[0] 是当前世界系绝对 yaw，叠加增量后即为世界系绝对目标角
        command.yaw = tools::limit_rad(spin_yaw_step + gimbal_pos[0]);
        // 这里的 pitch=0 是**有意的**：自旋搜索时保持平视，不要抬头/低头扫地面。
        // 与上面两种「漏填默认 0」的情况不同，control 在这里是 true。
        command.pitch = tools::limit_rad(0.0);
      } else {
        // 原地保持：gimbal_pos[0]/[1] 是世界系绝对 yaw/pitch，原样下发即为「保持当前角度」
        command.control = false;
        command.yaw = gimbal_pos[0];
        command.pitch = gimbal_pos[1];
      }

      main_recorder.record(img, q_main, timestamp);
      if (gimbal_wave_motion) apply_gimbal_wave();
      hold_current_pose();
      serial_board.send(command);
    } else {
      //这是主相机的处理，立马处理的目标
      // 过滤敌人
      decider.armor_filter(armors);

      // 【已停用】无法可靠判断兵种，暂停按兵种设置优先级
      // decider.set_priority(armors);

      //全向相机中拿出装甲板的候选（各相机独立 YOLO 推理，已按优先级排过序）
      auto detection_queue = omniperception::take_detection_results(pipeline);

      // 主相机已锁定目标时全向不再推理（ColorGate 收到 main_tracking 后暂停），
      // 但队列里可能还残留进入 tracking 之前的一条结果。直接丢弃，避免它按
      // 「优先级更高」把已经稳定的 tracking 拉回 switching。
      if (tracker.state() == "tracking") detection_queue.clear();

      decider.sort(detection_queue);

      // 传入的是全向相机中的装甲板候选，和主相机中的装甲板候选，为什么两个不放在同一个queue中传入给tracker呢？
      // 因为我们认为在主相机发现的相机的优先级会天然的大于全向相机中发现的
      // 【想象一下，主相机发现了一个可以轻易瞄准的目标，这个时候全向相机发现了一个理论上重要的目标，这个时候让云台转动去寻找那个目标，先不说准确性的问题，这样无疑是浪费时间的】
      auto [switch_target, target] = tracker.track(detection_queue, armors, timestamp, true, img.size());

      // 把主相机跟踪状态广播给各全向推理线程：tracking 时暂停全向 YOLO，
      // 只保留常驻颜色层。放在 track() 之后，丢失目标最多晚一帧恢复。
      omni_state->set_main_tracking(tracker.state() == "tracking");
      
      // 全向相机的 delta_yaw 是「大yaw 系」角（零点 = 大yaw 正前方、向左为正，
      // 由 Decider::delta_angle 用 tf 给的「相机 -> 大yaw」位姿算出），
      // 必须加大yaw 当前的世界系 yaw 才是下位机要求的世界系绝对角。
      // 注意：这里不能用 gimbal_pos[0]——那是小yaw 的 yaw，两个轴是独立的。
      if (tracker.state() == "switching") {
        // switch_target.armors 为空 = 已决定切换、但新目标还没观测到，此时它的
        // delta_yaw / delta_pitch 是 DetectionResult 的默认值 0。0x0402 报文里
        // 没有 control 位、pitch/yaw 会被固件无条件执行，直接下发 0 等于命令云台
        // 转到「pitch=0、世界系 yaw=0」。所以 control=false 时显式保持当前姿态。
        command.shoot = false;
        command.control = !switch_target.armors.empty();
        if (command.control) {
          command.pitch = tools::limit_rad(switch_target.delta_pitch);
          command.yaw = tools::limit_rad(switch_target.delta_yaw + omni_base_yaw);
        } else {
          command.pitch = gimbal_pos[1];
          command.yaw = gimbal_pos[0];
        }
      }

      else if (tracker.state() == "lost") {
        command = decider.decide(detection_queue);
        if (command.control) {
          // 全向有新检测：更新缓存，发送新值
          // decide() 返回的同样是大yaw 系角，需转成世界系绝对角后再下发
          command.yaw = tools::limit_rad(command.yaw + omni_base_yaw);
          cached_omni_yaw = command.yaw;
          cached_omni_pitch = command.pitch;
          has_omni_cache = true;
        } else if (has_omni_cache) {
          // 全向暂时丢失：回放缓存值，保持云台继续转向目标
          command.control = true;
          command.shoot = false;
          command.yaw = cached_omni_yaw;
          command.pitch = cached_omni_pitch;
        } else {
          // 既没有新的全向结果、也没有缓存：decider.decide() 返回的是
          // {false, false, 0, 0}，而 0x0402 的 pitch/yaw 会被固件无条件执行。
          // 直接下发会把云台拽回世界系 0 度，这里显式保持当前姿态。
          command.control = false;
          command.shoot = false;
          command.yaw = gimbal_pos[0];
          command.pitch = gimbal_pos[1];
        }
      }

      else {
        // aimer 输出的 yaw 已是 IMU 世界系绝对角（Aimer 内部对世界系 armor_xyza_list 取 atan2），
        // 与下位机的要求一致，直接下发，绝不能再叠加 gimbal_pos[0]。
        command = aimer.aim(target, timestamp, serial_board.bullet_speed);
        has_omni_cache = false;
      }

      command.shoot = shooter.shoot(command, aimer, target, gimbal_pos);
      if (gimbal_wave_motion) apply_gimbal_wave();

      // 绘制检测框和跟踪信息（用于赛后录像复盘）
      for (const auto & armor : armors) {
        cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);
        std::string label = fmt::format("{} {} {:.2f}",
          auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name], armor.confidence);
        cv::putText(img, label, cv::Point(armor.box.x, armor.box.y - 5),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        for (const auto & pt : armor.points) {
          cv::circle(img, pt, 2, cv::Scalar(0, 0, 255), -1);
        }
      }
      // HUD：状态 + IMM 的预测信息（距离 / 角速度 / 模型概率 / NIS 一致性）
      std::string hud =
        fmt::format("State: {} | Targets: {}", tracker.state(), target == nullptr ? 0 : 1);
      if (target != nullptr) {
        const Eigen::VectorXd x = target->ekf_x();
        if (x.size() >= 11) {
          const Eigen::Vector3d mu = target->model_probabilities();
          hud += fmt::format(
            " | d {:.2f}m w {:+.1f} mu {:.2f}/{:.2f}/{:.2f} nis {:.2f}", std::hypot(x[0], x[2]),
            x[7], mu[0], mu[1], mu[2], target->nis_ratio());
        }
      }
      cv::putText(
        img, hud, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

      main_recorder.record(img, q_main, timestamp);

  //     tools::logger()->info("yaw={:.1f}° gimbal={:.1f}° target={:.1f}°",
  // command.yaw * 57.3, gimbal_pos[0] * 57.3, (command.yaw - gimbal_pos[0]) * 57.3);
      hold_current_pose();
      serial_board.send(command);

      // //test//

      // sleep(5);
      // tools::logger()->info("sleeping```````");

      // //test//
    }
  }

  return 0;
}
