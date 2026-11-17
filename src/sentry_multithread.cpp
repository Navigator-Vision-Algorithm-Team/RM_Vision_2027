/*
 * 哨兵（双yaw + 多全向相机）自瞄主程序：把「主相机精瞄 + 全向补盲」合成一路云台指令。
 *
 * 数据流：
 *   相机：主相机 + 配置启用的全向相机（omni_camera_ids，旧键 omni_camera_count）
 *         → YoloPipeline（每路一个 YOLO 实例、
 *         一个绑核线程）→ 主相机结果优先的统一队列 → 本文件主循环。
 *   姿态：中转进程经 iceoryx2 共享内存发布 imu（500Hz）→ io::RelayClient 接收线程
 *         取样并插值 → ImuState（大yaw / 小yaw）→ 刷新 io::tf::Tree 的两个动态关节
 *         → Solver::set_R_gimbal2world。
 *   决策：Decider（全向候选排序）与 Tracker（主相机优先）→ Aimer / Shooter
 *         → io::Command → io::RelayClient::send → iceoryx2 cmd_aim 主题 → 中转写设备。
 *   外设：本进程不打开串口、不持有 CAN、不参与导航；裁判状态与机器人状态
 *         全部来自中转发布的 robot / game 主题。
 *
 * 线程：main() 主循环运行在进程主线程；推理在 YoloPipeline 的各相机线程；
 *       iceoryx2 收发在 io::RelayClient 的接收线程。本文件不跨线程共享可写状态，
 *       并发保护由 YoloPipeline / RelayClient 内部负责。
 *
 * 依赖：io::tf（外参来源优先级：tf 树 > yaml 的 omni_* 键 > 安装角近似，t 置 0）、
 *       iceoryx2（系统库，与中转共用同一份安装），配置示例 configs/sentry_example.yaml。
 *
 * 注意：
 *   1. 下发的 yaw/pitch 是世界系绝对角，由 Aimer 直接给出，不得再叠加 gimbal_pos[0]；
 *      Decider 输出的 delta_yaw 是「大yaw 系」角，必须先叠加 omni_base_yaw 才能下发。
 *   2. 链路失效（无 IMU 超时、契约版本不符、中转未启动）时 RelayClient 会把 shoot 位
 *      强制清零并让 can_shoot() 返回 false：云台继续跟踪，但绝不发弹。
 *   3. 本进程不再持有 ROS 2 节点，Exiter 无需再考虑 rclcpp 抢占 SIGINT 的顺序问题。
 */
#include <fmt/core.h>

#include <chrono>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/relay/relay_client.hpp"
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
#include "tasks/FST/sentry_fsm.hpp"
#include "tasks/omniperception/omni_selection.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  auto yaml = tools::load(config_path);

  // 录像参数：主相机出帧 120~150 fps，录像只按 record_fps_main 抽帧（默认 30），
  // 复盘够用，编码与磁盘都省 4/5；quality 对应 VAAPI global_quality / libx264 crf，
  // 取值 1~51，越小越清晰、体积越大
  const double main_record_fps =
    yaml["record_fps_main"] ? yaml["record_fps_main"].as<double>() : 30.0;
  const int record_quality = yaml["record_quality"] ? yaml["record_quality"].as<int>() : 25;
  tools::Recorder main_recorder(main_record_fps, "main", record_quality);

  // 坐标变换树：所有几何量（每个相机相对旋转轴的位置/姿态）都来自配置 tf.frames，
  // 代码里不写死任何安装尺寸。大yaw 与小yaw 作为两个动态关节，每帧由 0x0503 刷新。
  io::tf::Tree tf_tree = io::tf::load_tree(yaml);
  if (tf_tree.empty()) {
    tools::logger()->warn(
      "[Main] 配置里没有 tf.frames：主相机/全向相机将使用各自的回退外参（未标定则为安装角近似）");
  }

  // 中转客户端：视觉进程不再打开串口，也不再持有 ROS 2 节点。
  // 串口收发、导航速度订阅、云台关节与裁判状态发布全部由 info_relay 进程承担。
  io::RelayClient serial_board(config_path);

  tools::Exiter exiter;

  // 相机初始化
  io::Camera camera(config_path, "main");

  // 启用的全向相机编号：优先 omni_camera_ids，缺省回落到旧键 omni_camera_count。
  // 返回升序去重列表，支持 [1, 3] 这类任意组合，不再要求从 1 连续递增。
  const std::vector<int> omni_ids = omniperception::resolve_omni_camera_ids(yaml);

  // 单主相机（omni_ids 为空）时，丢失目标后的云台行为开关：
  // 没有有效瞄准角时由 tasks/spin_decision 统一执行 yaw 扫描 + pitch 点头，无开关
  // 状态机实例：跨帧保存当前状态，是本文件唯一的模式记忆
  sentry_fsm::Fsm fsm;
  // 缓冲时间， 确保全向相机初始化完成，避免出现空帧，但是我们的全向相机和主相机并不在同一个物理链路上，所以这个sleep并没有非常的必要
  // if (!omni_ids.empty()) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // }

  // 初始化全向相机
  std::vector<std::unique_ptr<io::Camera>> omni_cameras;
  std::vector<omniperception::OmniCameraConfig> omni_configs;

  for (size_t slot = 0; slot < omni_ids.size(); ++slot) {
    // i 是物理编号（决定 sursign 与全部 omni_*_i 配置键），slot 只是本次循环的下标
    const int i = omni_ids[slot];
    auto sursign = "omni" + std::to_string(i);
    auto cam = std::make_unique<io::Camera>(config_path, sursign);
    // 全向相机的实际出帧率来自 frame_rate_omniN（配置里是 15）：录制器若按 30 声明，
    // 15fps 的码流会被播放器按 30fps 播放，回放速度翻倍、时间轴对不上
    const double omni_record_fps =
      yaml["frame_rate_" + sursign] ? yaml["frame_rate_" + sursign].as<double>() : 15.0;
    auto recorder = std::make_unique<tools::Recorder>(omni_record_fps, sursign, record_quality);

    // 缓冲时间：只在还有下一路待初始化时等待，编号不连续也不影响
    if (slot + 1 < omni_ids.size()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    //配置设置
    omniperception::OmniCameraConfig cfg;
    cfg.camera = cam.get();
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
    // 按外参来源处理，避免"明明用了 tf 却报警说要用安装角"的误导：
    //   ① tf 查到          -> 使用 tf 外参，不输出日志
    //   ② tf 无、yaml 有    -> 使用 yaml 外参，不输出日志
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
      } else if (!cfg.has_extrinsic) {
        // ③ 两个来源都没有：decider 会退回 mount_to_R_camera2gimbal()，且 t 置 0。
        tools::logger()->warn(
          "[omni{}] tf 里没有 {} -> {}，且未配置 omni_R/t_camera2gimbal_{} "
          "-> 将用安装角 mount_yaw/mount_pitch 近似外参（t=0，近距离有视差误差）",
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

  const auto config_enemy_color =
    yaml["enemy_color"].as<std::string>() == "red" ? auto_aim::Color::red
                                                  : auto_aim::Color::blue;
  auto omni_state = std::make_shared<omniperception::OmniSharedState>();
  omni_state->set_enemy_color(config_enemy_color);

  // 敌我颜色以新通信协议转发的裁判 0x0201 robot_id 为准；未收到时使用配置兜底。
  auto refresh_enemy_color = [&]() {
    const auto own_red = serial_board.own_team_red();
    if (!own_red) return;
    const auto enemy_color = *own_red ? auto_aim::blue : auto_aim::red;
    tracker.set_enemy_color(enemy_color);
    decider.set_enemy_color(enemy_color);
    omni_state->set_enemy_color(enemy_color);
  };

  // 状态机需要回调刷新敌我颜色；先固化成 std::function，避免每帧拷贝捕获列表
  std::function<void()> refresh_fn = refresh_enemy_color;

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
    }
  }

    // 全向感知缓存：全向发现目标后的 yaw/pitch，主相机锁定前反复播报
  bool has_omni_cache = false;
  double cached_omni_yaw = 0;
  double cached_omni_pitch = 0;

  // 主相机最后出帧的时间：用于相机掉线时的告警（超时等待本身不会卡死主循环）
  auto last_main_frame = std::chrono::steady_clock::now();
  bool camera_stall_warning_logged = false;

  while (!exiter.exit()) {
    // 开赛前关闭推理：相机仍持续出图（主相机帧照常入队，供录像使用）
    pipeline.set_inference_enabled(serial_board.is_game_started());

    // 限时等待主相机结果：统一结果队列里主相机的优先级最高。
    // 必须带超时，相机掉线时也能回到 while 条件响应 Ctrl+C / SIGTERM。
    auto_aim::multithread::CameraResult frame;
    const auto status = pipeline.wait_main(frame, std::chrono::milliseconds(100));
    if (status == auto_aim::multithread::WaitStatus::Stopped) break;  // 推理池已停止

    if (status == auto_aim::multithread::WaitStatus::Timeout) {
      const auto now = std::chrono::steady_clock::now();
      const double stalled = std::chrono::duration<double>(now - last_main_frame).count();
      if (stalled >= 2.0 && !camera_stall_warning_logged) {
        camera_stall_warning_logged = true;
        tools::logger()->warn("[Main] 主相机已 {:.1f}s 没有新帧，继续等待...", stalled);
      }
      continue;
    }

    last_main_frame = std::chrono::steady_clock::now();
    camera_stall_warning_logged = false;
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
      main_recorder.record(img, q_main, timestamp);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // 主相机在小yaw 上，Solver 的 "gimbal" 就是小yaw 板
    //（R_gimbal2imubody 在当前配置里为单位阵，即 tf 的 small_yaw 与 IMU 参考系重合）
    solver.set_R_gimbal2world(q_main);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    if (serial_board.reset_pending()) {
      tracker.reset();
    }

    // ---- 状态机：由当前状态决定本帧走哪条瞄准路径 ----
    // 动作与转移都在 sentry_fsm 内部完成（每个状态函数末尾调用 check()），
    // 这里只负责组上下文、取结果、做统一收尾。
    sentry_fsm::Context fctx{
      img,                    // 主相机帧，只读
      timestamp,              // 采集时刻
      armors,                 // 主相机检测，状态机内部会按敌我颜色过滤
      gimbal_pos,             // 世界系绝对 yaw/pitch/roll，rad
      omni_base_yaw,          // 大yaw 世界系 yaw，rad
      !omni_ids.empty(),      // 是否启用全向相机
      pipeline,               // 取全向推理结果
      tracker,                // 目标跟踪
      decider,                // 全向候选排序与角度
      aimer,                  // 主相机弹道解算
      shooter,                // 开火判据
      serial_board,           // 取弹速与热量门限
      has_omni_cache,         // 全向缓存有效标志（跨帧）
      cached_omni_yaw,        // 缓存 yaw，rad（跨帧）
      cached_omni_pitch,      // 缓存 pitch，rad（跨帧）
      refresh_fn};            // 刷新敌我颜色的回调
    const bool main_path = fsm.tick(fctx);

    io::Command command = fctx.cmd;
    tools::VideoOverlay overlay;  // 录像覆盖层，由状态机路径填充

    if (main_path) {
      // 热量保护：枪管热量到 0x0201 上限前 heat_stop_margin 就停止开火
      //（其余控制量照常下发，云台继续跟瞄，只是不发弹）
      command.shoot =
        shooter.shoot(command, aimer, fctx.target, gimbal_pos) && serial_board.can_shoot();

      // 只组装"要画什么"的纯数据。限帧与绘制都在 Recorder 的保存线程内完成，
      // 被丢弃的帧不会付出任何绘制成本，也不占用主循环 CPU。
      for (const auto & armor : armors) {
        if (overlay.armor_count >= tools::VideoOverlay::kMaxArmors) break;
        auto & item = overlay.armors[overlay.armor_count++];
        item.box = armor.box;
        const size_t points = std::min<size_t>(4, armor.points.size());
        for (size_t k = 0; k < points; ++k) item.points[k] = armor.points[k];
        item.color = armor.color == auto_aim::Color::red ? cv::Scalar(0, 0, 255)
                                                      : cv::Scalar(255, 0, 0);
        const auto label = fmt::format("{} {} {:.2f}", auto_aim::COLORS[armor.color],
          auto_aim::ARMOR_NAMES[armor.name], armor.confidence);
        tools::set_overlay_text(item.label, item.label_len, sizeof(item.label), label);
      }
      auto hud = fmt::format("State: {} | FSM: {} | Targets: {}", tracker.state(),
        fsm.state_name(), fctx.target == nullptr ? 0 : 1);
      if (fctx.target != nullptr) {
        const Eigen::VectorXd x = fctx.target->ekf_x();
        if (x.size() >= 11) {
          const Eigen::Vector3d mu = fctx.target->model_probabilities();
          hud += fmt::format(
            " | d {:.2f}m w {:+.1f} mu {:.2f}/{:.2f}/{:.2f} nis {:.2f}", std::hypot(x[0], x[2]),
            x[7], mu[0], mu[1], mu[2], fctx.target->nis_ratio());
        }
      }
      tools::set_overlay_text(overlay.hud, overlay.hud_len, sizeof(overlay.hud), hud);
    }

    // ================== 本轮唯一的收尾点 ==================
    // 所有分支只负责算出 command，不在这里之外下发：
    //   · 指令已由状态机 finalize() 统一结算：无有效瞄准角时回填当前姿态并禁火；
    //   · 录像与下发集中在此，保证每轮循环最多一次，不会因分支重排而重复发送；
    //   · 上面的 continue（推理池超时/空帧/比赛未开始）在到达这里之前就跳出了，
    //     它们继续保持「只录像、不下发」的原有语义。
    main_recorder.record(img, q_main, timestamp, overlay);
    serial_board.send(command);
  }

  return 0;
}
