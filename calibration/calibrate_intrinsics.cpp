// =============================================================================
//  calibrate_intrinsics.cpp —— 相机内参标定工具（主相机 + 四个全向相机）
//
//  ---------------------------------------------------------------------------
//  一、这个程序是干什么的
//  ---------------------------------------------------------------------------
//  用一块棋盘格，标定相机的 **内参（焦距 / 光心）** 和 **畸变系数**。
//  标定结果直接粘贴到配置文件里（程序会打印好现成的 yaml 片段）。
//
//  本工具只做内参，不做"相机 → 转轴"的外参（那个要用转轴 + 固定靶做手眼标定，
//  见 calibration/README.md）。
//
//  ---------------------------------------------------------------------------
//  二、需要准备什么
//  ---------------------------------------------------------------------------
//  1) 棋盘格：本项目用的是 9×13 格、单格 30 mm 的板子（打印后贴在硬板上，务必平整）。
//     ⚠ 重要：OpenCV 数的是"内角点"，不是"格子"。
//         9×13 格的板子 → 内角点 8×12 → --cols 8 --rows 12
//       如果你不确定手上的板子到底是几×几格，先跑一次 --probe，程序会自动试几种规格。
//  2) 相机装到车上（不要拆下来标，拆装会改变内参之外的机械关系）。
//  3) 光照均匀，避免棋盘反光；曝光时间尽量 <= 5 ms，避免运动模糊。
//
//  ---------------------------------------------------------------------------
//  三、编译
//  ---------------------------------------------------------------------------
//      cmake -B build && cmake --build build -j$(nproc) --target calibrate_intrinsics
//
//  ---------------------------------------------------------------------------
//  四、怎么跑（由易到难）
//  ---------------------------------------------------------------------------
//  (0) 先确认棋盘规格能不能被检测到（不采集、不保存，10 秒出结果）：
//
//        ./build/calibrate_intrinsics configs/sentry_example.yaml --camera main --probe
//
//  (1) 标主相机：
//
//        ./build/calibrate_intrinsics configs/sentry_example.yaml \
//              --camera main --cols 8 --rows 12 --square-mm 30
//
//  (2) 标某一台全向相机（后缀是 omni1 / omni2 / omni3 / omni4，对应配置里的
//      camera_name_omniN）：
//
//        ./build/calibrate_intrinsics configs/sentry_example.yaml \
//              --camera omni1 --cols 8 --rows 12 --square-mm 30
//
//  (3) 一台一台依次标完五路（每标完一路会提示按回车继续下一路）：
//
//        ./build/calibrate_intrinsics configs/sentry_example.yaml \
//              --camera all --cols 8 --rows 12 --square-mm 30
//
//      说明：配置里没有 camera_name_omniN 的相机会被自动跳过（例如 omni_camera_count: 0）。
//
//  可选参数：
//      --out <dir>        图片与结果输出目录，默认 calibration/out
//      --rational         用有理畸变模型（k1..k6，8 个系数）。广角/全向相机建议开启
//      --cols / --rows    内角点列数 / 行数（默认 8 / 12）
//      --square-mm        单格边长（毫米，默认 30）
//      --from-dir <dir>   不连相机，直接对已保存的图片离线重算（改参数重跑很方便）
//
//      ⚠ 参数支持 "--key value" 和 "--key=value" 两种写法，顺序任意，配置文件路径放哪都行。
//
//  ---------------------------------------------------------------------------
//  五、采集时怎么操作（窗口里）
//  ---------------------------------------------------------------------------
//      [空格]  拍一张（只有检测到棋盘才会存）
//      [c]     开始计算内参（至少 15 张；建议 20~40 张）
//      [d]     删除上一张（拍糊了、检测歪了）
//      [q]     退出
//
//  窗口里绿色十字就是检测到的角点。**只拍绿十字正确覆盖在棋盘上的帧**，
//  检测不到时按空格不会存，程序会提示。
//
//  ---------------------------------------------------------------------------
//  六、怎么拍（决定精度的关键，别偷懒）
//  ---------------------------------------------------------------------------
//  · 数量：20~40 张；少于 15 张结果不可信。
//  · 姿态：棋盘要绕三个轴都倾斜（左右歪、上下俯仰、整体旋转），不能只是平移。
//  · 位置：棋盘要出现在画面的 **四个角 + 中心**，广角相机尤其要占满边缘。
//  · 大小：棋盘不要太小（至少占画面 1/4），也不要拍到出画（角点必须全在画面内）。
//  · 静止：按键前让手停稳，避免运动模糊。
//  · 全向相机（广角）：边缘畸变最大，必须在画面边缘也拍够，否则边缘目标会偏。
//
//  ---------------------------------------------------------------------------
//  七、结果怎么看、怎么用
//  ---------------------------------------------------------------------------
//  计算完成后会打印：
//    · 总体重投影误差 RMS：主相机 < 0.4 px 算好，全向相机 < 1.0 px 算好；
//    · 每张图的误差：某张 > 3 px 就是那张拍坏了，删掉重拍（程序会提示是哪几张）。
//  结果同时写进：
//    calibration/out/<main|omniN>/calib_result.yaml     （完整结果，留档）
//    calibration/out/<main|omniN>/view_XXX.png          （采集的原始图片）
//  并把可直接粘贴的片段打印到屏幕，例如主相机：
//
//      camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
//      distort_coeffs: [k1, k2, p1, p2, k3]
//
//  全向相机则是 omni_camera_matrix_N / omni_distort_coeffs_N（N 与 --camera 后缀一致）。
//  粘完建议再跑一次本工具点 [c] 前后对比 RMS，确认配置写对了。
//
//  ---------------------------------------------------------------------------
//  八、常见问题
//  ---------------------------------------------------------------------------
//  · 一直检测不到棋盘：
//      ① 规格填错（格子数 vs 内角点），用 --probe 试；
//      ② 棋盘反光/太暗，换角度或补光；
//      ③ 棋盘出画或太小。
//  · RMS 很大（> 1.5 px）：
//      ① 有糊的/角点歪的图，删掉；
//      ② 姿态太单一（全是正面拍），补倾斜角度；
//      ③ 广角相机没开 --rational。
//  · 提示 "camera_name_xxx 是 fake"：
//      说明配置里该相机是虚拟相机，标出来的内参没有意义，换真机配置。
//  · 配置文件里相机键名：main → camera_name_main，omni1 → camera_name_omni1 ……
//  · 程序打不开相机：确认没有别的程序占用（正在跑的自瞄程序要先关掉）。
// =============================================================================

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace
{

struct Args
{
  std::string config = "configs/sentry_example.yaml";
  std::string camera = "main";
  int cols = 8;
  int rows = 12;
  double square_mm = 30.0;
  std::string out = "calibration/out";
  std::string from_dir;  // 非空：不采集，直接对目录里已有的图片重算
  bool rational = false;
  bool probe = false;
  bool help = false;
};

void print_help()
{
  std::cout
    << "用法：\n"
    << "  calibrate_intrinsics <配置文件> --camera <main|omni1..4|all> [选项]\n\n"
    << "示例：\n"
    << "  calibrate_intrinsics configs/sentry_example.yaml --camera main --cols 8 --rows 12\n"
    << "  calibrate_intrinsics configs/sentry_example.yaml --camera omni1 --rational\n"
    << "  calibrate_intrinsics configs/sentry_example.yaml --camera all\n"
    << "  calibrate_intrinsics configs/sentry_example.yaml --camera main --probe\n\n"
    << "选项（--key value 与 --key=value 两种写法都支持）：\n"
    << "  -c, --camera <name>   相机后缀：main / omni1 / omni2 / omni3 / omni4 / all（默认 main）\n"
    << "      --cols <n>        内角点列数（默认 8；注意 9×13 的板子是 8×12）\n"
    << "      --rows <n>        内角点行数（默认 12）\n"
    << "      --square-mm <x>   单格边长，毫米（默认 30）\n"
    << "      --out <dir>       图片与结果输出目录（默认 calibration/out）\n"
    << "      --from-dir <dir>  不连相机，直接对目录里已采集的图片重算（离线重标定）\n"
    << "      --rational        用有理畸变模型 k1..k6（广角/全向相机建议开启）\n"
    << "      --probe           只探测棋盘规格，不采集\n"
    << "  -h, --help            打印本帮助\n";
}

// 手写解析：兼容 "--key value" 与 "--key=value"（OpenCV 自带的解析器不认前者）
Args parse_args(int argc, char ** argv)
{
  Args a;
  bool config_taken = false;

  for (int i = 1; i < argc; ++i) {
    const std::string token = argv[i];

    if (token == "-h" || token == "--help") {
      a.help = true;
      continue;
    }
    if (token == "--probe") {
      a.probe = true;
      continue;
    }
    if (token == "--rational") {
      a.rational = true;
      continue;
    }

    const auto eq = token.find('=');
    const std::string key = token.substr(0, eq);
    const auto value = [&]() -> std::string {
      if (eq != std::string::npos) return token.substr(eq + 1);
      if (i + 1 >= argc) throw std::runtime_error("参数 " + key + " 需要一个值");
      return argv[++i];
    };

    if (key == "-c" || key == "--camera") {
      a.camera = value();
    } else if (key == "--cols") {
      a.cols = std::stoi(value());
    } else if (key == "--rows") {
      a.rows = std::stoi(value());
    } else if (key == "--square-mm") {
      a.square_mm = std::stod(value());
    } else if (key == "--out") {
      a.out = value();
    } else if (key == "--from-dir") {
      a.from_dir = value();
    } else if (!token.empty() && token[0] != '-') {
      if (!config_taken) {
        a.config = token;
        config_taken = true;
      }
    } else {
      throw std::runtime_error("未知参数: " + token + "（用 --help 看用法）");
    }
  }
  return a;
}

struct Session
{
  std::string suffix;
  std::vector<std::vector<cv::Point2f>> image_points;
  std::string dir;
};

// 棋盘在相机坐标系下的 3D 点（z = 0），单位与 square_mm 一致
std::vector<cv::Point3f> board_object_points(int cols, int rows, double square)
{
  std::vector<cv::Point3f> obj;
  obj.reserve(static_cast<size_t>(cols * rows));
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      obj.emplace_back(static_cast<float>(x * square), static_cast<float>(y * square), 0.0F);
    }
  }
  return obj;
}

bool find_board(
  const cv::Mat & gray, const cv::Size & pattern, std::vector<cv::Point2f> & corners)
{
  corners.clear();
  const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  if (!cv::findChessboardCorners(gray, pattern, corners, flags)) return false;

  // 亚像素精化：角点精度直接决定内参精度
  cv::cornerSubPix(
    gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));
  return true;
}

// 试几种常见的棋盘规格，帮用户确认自己手上是哪一种
void probe_patterns(io::Camera & camera, int cols, int rows)
{
  std::cout << "\n[probe] 正在尝试几种棋盘规格（每次取 10 帧，只要能检测到就算通过）...\n";
  std::vector<cv::Size> candidates = {
    {cols, rows}, {rows, cols}, {8, 12}, {12, 8}, {9, 13}, {7, 11}, {11, 7}, {6, 9}, {9, 6}};
  std::sort(candidates.begin(), candidates.end(), [](const cv::Size & a, const cv::Size & b) {
    return a.width * a.height > b.width * b.height;
  });
  candidates.erase(
    std::unique(candidates.begin(), candidates.end(), [](const cv::Size & a, const cv::Size & b) {
      return a.width == b.width && a.height == b.height;
    }),
    candidates.end());

  cv::Mat img;
  std::chrono::steady_clock::time_point ts;
  for (int i = 0; i < 10; ++i) camera.read(img, ts);  // 丢前几帧，等曝光稳定

  std::vector<cv::Size> passed;
  for (const auto & p : candidates) {
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;
    if (find_board(gray, p, corners)) passed.push_back(p);
  }

  if (passed.empty()) {
    std::cout << "[probe] 没有检测到任何规格。请检查：棋盘是否在画面内、是否太小、是否反光。\n";
    return;
  }
  std::cout << "[probe] 检测通过的内角点规格（列 x 行）：\n";
  for (const auto & p : passed) {
    std::cout << "        " << p.width << " x " << p.height << "   →  --cols " << p.width
              << " --rows " << p.height << "\n";
  }
  std::cout << "[probe] 说明：一块板子只应有一种真实规格，出现多种通常是其中一种误检；"
               "请用能稳定检测到的那一组。\n";
}

// 用已采集的角点解算内参，打印结果并写文件；成功返回 true
bool calibrate_and_report(
  Session & s, const cv::Size & image_size, int cols, int rows, double square_mm, bool rational)
{
  if (s.image_points.size() < 15) {
    std::cout << "[warn] 只采到 " << s.image_points.size() << " 张，至少 15 张才能算。继续采集（空格）。\n";
    return false;
  }

  const auto object_points_all = [&] {
    std::vector<std::vector<cv::Point3f>> v(s.image_points.size());
    const auto one = board_object_points(cols, rows, square_mm);
    for (auto & e : v) e = one;
    return v;
  }();

  cv::Mat K, D;
  std::vector<cv::Mat> rvecs, tvecs;
  int flags = 0;
  if (rational) flags |= cv::CALIB_RATIONAL_MODEL;

  double rms = 0.0;
  try {
    rms = cv::calibrateCamera(
      object_points_all, s.image_points, image_size, K, D, rvecs, tvecs, flags);
  } catch (const cv::Exception & e) {
    std::cout << "[error] calibrateCamera 失败：" << e.what() << "\n";
    return false;
  }

  // 每张图的重投影误差，用来剔除坏图
  std::vector<std::pair<int, double>> per_view;
  for (size_t i = 0; i < s.image_points.size(); ++i) {
    std::vector<cv::Point2f> proj;
    cv::projectPoints(object_points_all[i], rvecs[i], tvecs[i], K, D, proj);
    double sum = 0.0;
    for (size_t k = 0; k < proj.size(); ++k) sum += cv::norm(proj[k] - s.image_points[i][k]);
    per_view.emplace_back(static_cast<int>(i), sum / static_cast<double>(proj.size()));
  }

  std::cout << "\n==================== 标定结果 (" << s.suffix << ") ====================\n";
  std::cout << "图像尺寸      : " << image_size.width << " x " << image_size.height << "\n";
  std::cout << "棋盘内角点    : " << cols << " x " << rows << "，格长 " << square_mm << " mm\n";
  std::cout << "使用图片      : " << s.image_points.size() << " 张\n";
  std::cout << "总体重投影 RMS: " << rms << " px"
            << (rms < 0.5 ? "  (很好)" : (rms < 1.0 ? "  (可接受)" : "  (偏大，建议删掉坏图)"))
            << "\n";

  std::vector<int> bad;
  for (const auto & [idx, err] : per_view) {
    if (err > 3.0) bad.push_back(idx);
  }
  if (!bad.empty()) {
    std::cout << "误差 > 3 px 的图片（建议删除重拍）：";
    for (int i : bad) std::cout << " view_" << cv::format("%03d", i);
    std::cout << "\n";
  }

  // ---- 输出：完整结果 + 可直接粘贴的配置片段 ----
  std::filesystem::create_directories(s.dir);
  const auto yaml_path = std::filesystem::path(s.dir) / "calib_result.yaml";
  std::ofstream ofs(yaml_path);
  ofs << "# 由 calibration/calibrate_intrinsics.cpp 生成\n";
  ofs << "camera: " << s.suffix << "\n";
  ofs << "image_size: [" << image_size.width << ", " << image_size.height << "]\n";
  ofs << "pattern_cols: " << cols << "\npattern_rows: " << rows << "\n";
  ofs << "square_mm: " << square_mm << "\n";
  ofs << "rms_px: " << rms << "\n";
  ofs << "views: " << s.image_points.size() << "\n";

  std::cout << "\n------------------------- 粘贴到配置里 -------------------------\n";
  std::string key_K = "camera_matrix";
  std::string key_D = "distort_coeffs";
  if (s.suffix.rfind("omni", 0) == 0) {
    const std::string n = s.suffix.substr(4);  // "omni3" -> "3"
    key_K = "omni_camera_matrix_" + n;
    key_D = "omni_distort_coeffs_" + n;
  }

  const auto list = [](const cv::Mat & m) {
    std::string out = "[";
    const int total = static_cast<int>(m.total());
    for (int i = 0; i < total; ++i) {
      out += cv::format(i ? ", %.10g" : "%.10g", m.at<double>(i));
    }
    return out + "]";
  };
  std::cout << key_K << ": " << list(K) << "\n";
  std::cout << key_D << ": " << list(D) << "\n";

  ofs << key_K << ": " << list(K) << "\n";
  ofs << key_D << ": " << list(D) << "\n";
  ofs.close();
  std::cout << "---------------------------------------------------------------\n";
  std::cout << "完整结果已写入: " << yaml_path.string() << "\n";
  std::cout << "原始图片目录  : " << s.dir << "\n\n";
  return true;
}

// 采一个后缀的相机，返回 false 表示这台相机打不开/被跳过
bool run_one_camera(
  const std::string & config_path, const std::string & suffix, int cols, int rows,
  double square_mm, const std::string & out_root, bool rational, bool probe_only)
{
  std::unique_ptr<io::Camera> camera;
  try {
    camera = std::make_unique<io::Camera>(config_path, suffix);
  } catch (const std::exception & e) {
    std::cout << "[skip] 打开相机 " << suffix << " 失败：" << e.what() << "\n";
    return false;
  }

  // 配置里是 fake/replay 相机时提醒：标出来的内参没有物理意义
  try {
    auto yaml = tools::load(config_path);
    const std::string key = "camera_name_" + suffix;
    const std::string name = yaml[key] ? yaml[key].as<std::string>() : std::string("(未配置)");
    if (name == "fake" || name == "replay") {
      std::cout << "[warn] " << key << " = " << name
                << "，这是虚拟相机，标定结果没有意义。请换成真机配置。\n";
    }
  } catch (const std::exception &) {
  }

  if (probe_only) {
    probe_patterns(*camera, cols, rows);
    return true;
  }

  const cv::Size pattern(cols, rows);
  const std::string title = "calibrate_intrinsics - " + suffix;
  std::cout << "\n==================== 开始采集 (" << suffix << ") ====================\n"
            << "空格=拍照  c=计算  d=删除上一张  q=退出   棋盘内角点 " << cols << "x" << rows
            << "，格长 " << square_mm << " mm\n";

  Session session;
  session.suffix = suffix;
  session.dir = (std::filesystem::path(out_root) / suffix).string();
  std::filesystem::create_directories(session.dir);

  cv::Mat img;
  std::chrono::steady_clock::time_point ts;
  for (int i = 0; i < 10; ++i) camera->read(img, ts);  // 等曝光/白平衡稳定

  bool calibrated = false;
  while (true) {
    camera->read(img, ts);
    if (img.empty()) continue;

    cv::Mat gray, vis;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;
    const bool found = find_board(gray, pattern, corners);

    cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
    if (found) cv::drawChessboardCorners(vis, pattern, corners, found);

    // 左上角状态栏
    const std::string status =
      cv::format("%s | %s | captured %zu", suffix.c_str(), found ? "BOARD OK" : "no board",
                 session.image_points.size());
    cv::putText(
      vis, status, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
      found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
    cv::putText(
      vis, "SPACE=capture  c=calibrate  d=undo  q=quit", cv::Point(12, 56),
      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);

    // 大图缩小显示，但检测与保存都用原图
    cv::Mat show = vis;
    if (vis.cols > 1400) {
      const double s = 1400.0 / vis.cols;
      cv::resize(vis, show, cv::Size(), s, s);
    }
    cv::imshow(title, show);

    const int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;

    if (key == ' ') {
      if (!found) {
        std::cout << "[info] 没检测到棋盘，未保存（检查规格/角度/光照）\n";
        continue;
      }
      session.image_points.push_back(corners);
      const auto path =
        (std::filesystem::path(session.dir) /
         cv::format("view_%03d.png", static_cast<int>(session.image_points.size()) - 1))
          .string();
      cv::imwrite(path, img);
      std::cout << "[capture] " << session.image_points.size() << " 张 -> " << path << "\n";
    }

    if (key == 'd') {
      if (!session.image_points.empty()) {
        session.image_points.pop_back();
        const auto path = (std::filesystem::path(session.dir) /
                           cv::format("view_%03d.png", static_cast<int>(session.image_points.size())))
                            .string();
        std::filesystem::remove(path);
        std::cout << "[undo] 已删除，当前 " << session.image_points.size() << " 张\n";
      }
    }

    if (key == 'c') {
      calibrated = calibrate_and_report(
        session, img.size(), cols, rows, square_mm, rational);
      if (calibrated) {
        tools::logger()->info("[calibrate] {} 完成", suffix);
        std::cout << "按任意键继续（下一台相机或退出）...\n";
        cv::waitKey(0);
        break;
      }
    }
  }

  cv::destroyWindow(title);
  return calibrated;
}

}  // namespace

int main(int argc, char * argv[])
{
  Args args;
  try {
    args = parse_args(argc, argv);
  } catch (const std::exception & e) {
    std::cout << "[error] " << e.what() << "\n\n";
    print_help();
    return 1;
  }
  if (args.help || argc <= 1) {
    print_help();
    return 0;
  }

  const auto & config_path = args.config;
  const auto & camera = args.camera;
  const int cols = args.cols;
  const int rows = args.rows;
  const double square_mm = args.square_mm;
  const auto & out_root = args.out;
  const bool rational = args.rational;
  const bool probe_only = args.probe;

  if (cols < 3 || rows < 3) {
    std::cout << "[error] --cols/--rows 至少 3（通常 8x12 或 9x13）\n";
    return 1;
  }

  std::cout << "配置      : " << config_path << "\n"
            << "棋盘内角点: " << cols << " x " << rows << "（格长 " << square_mm << " mm）\n"
            << "畸变模型  : " << (rational ? "rational (k1..k6)" : "standard (k1,k2,p1,p2,k3)")
            << "\n";

  // ---------- 离线重算：不连相机，直接读目录里的图片 ----------
  if (!args.from_dir.empty()) {
    std::vector<std::string> files;
    for (const auto & entry : std::filesystem::directory_iterator(args.from_dir)) {
      if (!entry.is_regular_file()) continue;
      const auto ext = entry.path().extension().string();
      if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") files.push_back(entry.path().string());
    }
    std::sort(files.begin(), files.end());
    if (files.empty()) {
      std::cout << "[error] " << args.from_dir << " 里没有 png/jpg 图片\n";
      return 1;
    }

    Session session;
    session.suffix = (camera == "all") ? std::string("main") : camera;
    session.dir = args.from_dir;

    cv::Size image_size;
    for (const auto & f : files) {
      cv::Mat gray = cv::imread(f, cv::IMREAD_GRAYSCALE);
      if (gray.empty()) continue;
      if (image_size.empty()) image_size = gray.size();
      std::vector<cv::Point2f> corners;
      if (find_board(gray, cv::Size(cols, rows), corners)) {
        session.image_points.push_back(corners);
      } else {
        std::cout << "[warn] 未检测到棋盘，跳过: " << f << "\n";
      }
    }
    std::cout << "[from-dir] 读到 " << session.image_points.size() << "/" << files.size()
              << " 张可用图片\n";
    return calibrate_and_report(session, image_size, cols, rows, square_mm, rational) ? 0 : 1;
  }

  std::vector<std::string> suffixes;
  if (camera == "all") {
    suffixes = {"main", "omni1", "omni2", "omni3", "omni4"};
    // 只保留配置里真实存在的相机
    auto yaml = tools::load(config_path);
    std::vector<std::string> exist;
    for (const auto & s : suffixes) {
      if (yaml["camera_name_" + s]) exist.push_back(s);
    }
    suffixes = exist;
    std::cout << "依次标定    : ";
    for (const auto & s : suffixes) std::cout << s << " ";
    std::cout << "\n";
    if (suffixes.empty()) {
      std::cout << "[error] 配置里没有任何 camera_name_main / camera_name_omniN\n";
      return 1;
    }
  } else {
    suffixes = {camera};
  }

  int ok = 0;
  for (const auto & s : suffixes) {
    if (run_one_camera(config_path, s, cols, rows, square_mm, out_root, rational, probe_only)) ++ok;
  }

  std::cout << "\n完成 " << ok << "/" << suffixes.size() << " 台相机的标定。\n"
            << "结果在 " << out_root << "/<相机>/calib_result.yaml，"
               "粘贴片段见上面的输出。\n";
  return ok > 0 ? 0 : 1;
}
