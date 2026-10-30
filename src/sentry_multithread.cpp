#include <fmt/core.h>

#include <chrono>
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

  // ROS2 初始化：内部启动独立线程，收到导航数据后立即经串口下发，
  // 与主循环（视觉处理）解耦，避免被主链路延迟。
  io::ROS2 ros2([&serial_board](const io::NavCommand & nav_command) {
    serial_board.send(nav_command);
  });

  // 云台关节反馈：复用 SerialBoard 已有的串口接收线程，收到一帧 0x502 就发布一次
  // JointState，不额外启动程序、不重复占用串口。
  serial_board.set_gimbal_joint_callback(ros2.gimbal_joint_sender());

  // Exiter 必须在 io::ROS2 之后构造：rclcpp::init 会用自己的 handler 接管 SIGINT，
  // 后安装的 handler 才能让 Ctrl+C 重新置位退出标志。
  tools::Exiter exiter;

  tools::logger()->info("[Main] Opening camera...");
  // 相机初始化
  io::Camera camera(config_path, "main");
  tools::logger()->info("[Main] Camera ready: {}", camera.device_name());

  int omni_count = 0;
  if (yaml["omni_camera_count"]) omni_count = yaml["omni_camera_count"].as<int>();

  // 单主相机（omni_count == 0）时，丢失目标后的云台行为开关：
  //   true  = 自旋扫描（旧行为，按固定步长持续转动）
  //   false = 原地保持当前朝向
  // 缺省 true，保持旧行为；yaml 里写 spin_when_no_omni: false 即可关闭自旋
  const bool spin_when_no_omni =
    yaml["spin_when_no_omni"] ? yaml["spin_when_no_omni"].as<bool>() : true;
  if (omni_count == 0) {
    tools::logger()->info(
      "[Main] single main camera, spin when lost: {}", spin_when_no_omni ? "on" : "off");
  }

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
    camera_jobs.push_back(std::move(main_job));

    auto omni_jobs = omniperception::make_omni_jobs(omni_configs, config_path);
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

    // 全向感知缓存：全向发现目标后的 yaw/pitch，主相机锁定前反复播报
  bool has_omni_cache = false;
  double cached_omni_yaw = 0;
  double cached_omni_pitch = 0;

  // 主相机最后出帧的时间：用于相机掉线时的告警（超时等待本身不会卡死主循环）
  auto last_main_frame = std::chrono::steady_clock::now();
  auto last_stall_log = last_main_frame;

  // rclcpp::init 还接管了 SIGTERM（只置 rclcpp 的 shutdown 标志），
  // 因此除 Exiter 的 SIGINT 外再检查 rclcpp::ok()，保证 Ctrl+C / kill 都能退出。
  while (!exiter.exit() && rclcpp::ok()) {
    // 开赛前关闭推理：相机仍持续出图（主相机帧照常入队，供录像使用），
    // 避免赛前 5 路 YOLO 空转占满 CPU。
    pipeline.set_inference_enabled(serial_board.is_game_started());

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

    // 无全向相机且主相机丢失目标：由 spin_when_no_omni 决定「自旋扫描」还是「原地保持」。
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
        command.pitch = tools::limit_rad(0.0);
      } else {
        // 原地保持：gimbal_pos[0]/[1] 是世界系绝对 yaw/pitch，原样下发即为「保持当前角度」
        command.control = false;
        command.yaw = gimbal_pos[0];
        command.pitch = gimbal_pos[1];
      }

      main_recorder.record(img, q_main, timestamp);
      serial_board.send(command);
    } else {
      //这是主相机的处理，立马处理的目标
      // 过滤敌人
      decider.armor_filter(armors);

      // 【已停用】无法可靠判断兵种，暂停按兵种设置优先级
      // decider.set_priority(armors);

      //全向相机中拿出装甲板的候选（各相机独立 YOLO 推理，已按优先级排过序）
      auto detection_queue = omniperception::take_detection_results(pipeline);

      decider.sort(detection_queue);

      // 传入的是全向相机中的装甲板候选，和主相机中的装甲板候选，为什么两个不放在同一个queue中传入给tracker呢？
      // 因为我们认为在主相机发现的相机的优先级会天然的大于全向相机中发现的
      // 【想象一下，主相机发现了一个可以轻易瞄准的目标，这个时候全向相机发现了一个理论上重要的目标，这个时候让云台转动去寻找那个目标，先不说准确性的问题，这样无疑是浪费时间的】
      auto [switch_target, targets] = tracker.track(detection_queue, armors, timestamp);
      
      // 全向相机的 delta_yaw 是「大yaw 系」角（零点 = 大yaw 正前方、向左为正，
      // 由 Decider::delta_angle 用 tf 给的「相机 -> 大yaw」位姿算出），
      // 必须加大yaw 当前的世界系 yaw 才是下位机要求的世界系绝对角。
      // 注意：这里不能用 gimbal_pos[0]——那是小yaw 的 yaw，两个轴是独立的。
      if (tracker.state() == "switching") {
        command.control = switch_target.armors.empty() ? false : true;
        command.shoot = false;
        command.pitch = tools::limit_rad(switch_target.delta_pitch);
        command.yaw = tools::limit_rad(switch_target.delta_yaw + omni_base_yaw);
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
        }
      }

      else {
        // aimer 输出的 yaw 已是 IMU 世界系绝对角（Aimer 内部对世界系 armor_xyza_list 取 atan2），
        // 与下位机的要求一致，直接下发，绝不能再叠加 gimbal_pos[0]。
        command = aimer.aim(targets, timestamp, serial_board.bullet_speed);
        has_omni_cache = false;
      }

      command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

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
      cv::putText(img,
        fmt::format("State: {} | Targets: {}", tracker.state(), targets.size()),
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

      main_recorder.record(img, q_main, timestamp);

  //     tools::logger()->info("yaw={:.1f}° gimbal={:.1f}° target={:.1f}°",
  // command.yaw * 57.3, gimbal_pos[0] * 57.3, (command.yaw - gimbal_pos[0]) * 57.3);
      serial_board.send(command);

      // //test//

      // sleep(5);
      // tools::logger()->info("sleeping```````");

      // //test//
    }
  }

  return 0;
}
