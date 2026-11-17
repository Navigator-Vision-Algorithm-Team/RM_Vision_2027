/*
 * 录制器实现：把相机帧与当帧 IMU 四元数异步落盘，视频走 FFmpeg 硬件 H.264 编码。
 *
 * 数据流：调用方 record(img, q, t) → 单槽队列 queue_（容量 1）→ saving_thread_
 *         → Recorder::Impl（BGR → sws 转 NV12 → 上传 VAAPI 表面 → h264_vaapi
 *         → Matroska 封装）→ records/<file_name>/<启动时刻>.mkv 与同名 .txt。
 *
 * 线程：record() 在调用方线程；Impl 的全部调用都在 saving_thread_ 内。全进程用
 *       g_encode_mutex 串行化编码，保证同一时刻只有一路在占用 GPU 的编码单元。
 *
 * 注意：功能 / 输入 / 输出等契约写在 tools/recorder.hpp，本文件只写实现要点。
 */
#include "recorder.hpp"

#include <fmt/chrono.h>

// FFmpeg 是纯 C 库，必须按 C 链接。
// 即使头文件自带 extern "C" 保护，这里仍显式包一层：少了它，编译出的目标文件引用的是
// C++ 修饰名（_Z14av_frame_allocv），而 libav*.so 导出的是未修饰的 C 名（av_frame_alloc），
// 链接期就会报大量 "undefined reference to `av_frame_alloc()'" —— 注意报错里带括号，
// 那就是 C++ 修饰名被反修饰的结果，而不是真的缺库。
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"

#include <cerrno>
#include <cstring>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace tools
{
namespace
{
// 功能：把 FFmpeg 的错误码翻译成可读文本，用于失败路径打印具体原因
// 输入：
//   err  FFmpeg 返回的负错误码（AVERROR_*），允许为 0
// 输出：可读字符串；无法识别时返回错误码的十进制形式
// 备注：无副作用；只在初始化失败时调用，不进入热路径
std::string av_error_text(int err)
{
  char buf[AV_ERROR_MAX_STRING_SIZE] = {0};
  if (av_strerror(err, buf, sizeof(buf)) < 0) return std::to_string(err);
  return buf;
}

// 全进程串行：多路相机各有一个保存线程，同一时刻只允许一个进入编码与写盘。
// 理由：5 路同时提交会让 GPU 的编码单元排队，实测串行（主相机 30fps + 4 路全向 15fps）
// 总计只占单核约 0.2，串行的代价远小于并发时的排队与抖动。
std::mutex g_encode_mutex;
}  // namespace

// 编码器 + 封装器的全部资源。对外不可见，只在保存线程内使用。
struct Recorder::Impl
{
  bool hardware = false;              // true = VAAPI 硬编；false = libx264 软编回退
  AVBufferRef * hw_device = nullptr;  // VAAPI 设备上下文；软编时为 nullptr
  AVCodecContext * ctx = nullptr;     // 编码器上下文
  AVFormatContext * fmt = nullptr;    // Matroska 封装器
  AVStream * stream = nullptr;        // 输出流
  AVFrame * sw_frame = nullptr;       // 软件帧：硬编用 NV12，软编用 YUV420P
  AVPacket * pkt = nullptr;           // 复用的码流包缓冲
  SwsContext * sws = nullptr;         // BGR24 -> sw_format 色彩转换
  AVPixelFormat sw_format = AV_PIX_FMT_NV12;  // 软件帧像素格式，由编码器决定
  int width = 0;                              // 编码宽度，px；取自首帧
  int height = 0;                             // 编码高度，px；取自首帧
  int64_t first_pts_ms = -1;                  // 首帧时间戳，ms；用于把码流起点归零
  bool header_written = false;                // 容器文件头是否已写入；未写入时禁止 write_trailer

  // 功能：释放编码器与封装器的全部资源
  // 输入：无
  // 输出：无
  // 备注：close() 会把所有指针置空，因此本析构与显式 close() 可以先后执行；
  //       open() 中途失败时，已分配的半成品资源也由这里兜底释放
  ~Impl() { close(); }

  // 功能：打开编码器与封装器，硬编优先、软编兜底
  // 输入：
  //   path     输出文件路径，扩展名 .mkv
  //   w / h    编码宽高，px，必须为偶数（NV12 / YUV420P 的色彩平面按 2x2 采样）
  //   fps      标称帧率，Hz，> 0
  //   quality  编码质量 1 ~ 51，越小越清晰
  // 输出：true 表示可编码；false 表示无可用编码器或封装器打开失败
  // 备注：失败时内部已释放部分资源但仍需调用 close() 兜底；只调用一次
  bool open(const std::string & path, int w, int h, int fps, int quality)
  {
    width = w;
    height = h;

    static std::once_flag log_once;
    std::call_once(log_once, [] { av_log_set_level(AV_LOG_ERROR); });

    if (!open_hardware(fps, quality)) {
      tools::logger()->warn(
        "[Recorder] h264_vaapi 不可用，回退 libx264 软编（CPU 占用会明显上升，录像质量不变）");
      avcodec_free_context(&ctx);
      if (hw_device) av_buffer_unref(&hw_device);
      if (!open_software(fps, quality)) {
        tools::logger()->error("[Recorder] libx264 也不可用，本次不录像");
        return false;
      }
    }
    return open_muxer(path);
  }

  // 功能：把一帧 BGR 图像编码并写入容器
  // 输入：
  //   bgr     待编码图像，CV_8UC3 或 CV_8UC1；尺寸必须与 open() 时一致
  //   pts_ms  该帧的采集时刻，ms，相对录制起点；内部会减去首帧时刻归零
  // 输出：true 表示本帧已成功交给编码器并完成写盘；false 表示本帧被丢弃
  // 备注：内部可能阻塞（硬件编码与磁盘写入），只在保存线程调用
  bool encode(const cv::Mat & bgr, int64_t pts_ms)
  {
    if (!ctx || !sws || bgr.empty()) return false;
    // 尺寸变化不受支持：编码器在 open() 时就固定了宽高，换分辨率只能重开文件
    if (bgr.cols != width || bgr.rows != height) return false;

    cv::Mat src;
    if (bgr.type() == CV_8UC3) {
      src = bgr;
    } else if (bgr.type() == CV_8UC1) {
      cv::cvtColor(bgr, src, cv::COLOR_GRAY2BGR);
    } else {
      return false;
    }

    if (av_frame_make_writable(sw_frame) < 0) return false;

    const uint8_t * src_data[4] = {src.data, nullptr, nullptr, nullptr};
    int src_linesize[4] = {static_cast<int>(src.step), 0, 0, 0};
    sws_scale(sws, src_data, src_linesize, 0, height, sw_frame->data, sw_frame->linesize);

    if (first_pts_ms < 0) first_pts_ms = pts_ms;
    const int64_t pts = pts_ms - first_pts_ms;

    // 硬编要把软件帧上传到 VAAPI 表面，软编可以直接送 NV12/YUV420P 软件帧
    auto submit = [&]() -> int {
      if (!hardware) {
        sw_frame->pts = pts;
        return avcodec_send_frame(ctx, sw_frame);
      }
      AVFrame * hw_frame = av_frame_alloc();
      if (!hw_frame) return AVERROR(ENOMEM);
      if (
        av_hwframe_get_buffer(ctx->hw_frames_ctx, hw_frame, 0) < 0 ||
        av_hwframe_transfer_data(hw_frame, sw_frame, 0) < 0) {
        av_frame_free(&hw_frame);
        return AVERROR(ENOMEM);
      }
      hw_frame->pts = pts;
      const int ret = avcodec_send_frame(ctx, hw_frame);
      av_frame_free(&hw_frame);  // send 内部已取引用，这里释放本地句柄
      return ret;
    };

    int ret = submit();
    // 编码器内部队列满：先排空再重试一次，仍失败则丢弃本帧
    if (ret == AVERROR(EAGAIN)) {
      if (!drain()) return false;
      ret = submit();
    }
    if (ret < 0) return false;

    return drain();
  }

  // 功能：冲刷编码器缓冲并关闭封装器，保证文件尾部完整
  // 输入：无
  // 输出：无
  // 备注：必须在保存线程 join 之后调用；重复调用安全
  void close()
  {
    if (ctx) {
      avcodec_send_frame(ctx, nullptr);  // 通知编码器没有更多帧，让它吐出缓冲的帧
      drain();
    }

    if (fmt) {
      // 文件头没写成功时调用 write_trailer 会踩空（初始化失败走了析构路径），必须判据保护
      if (header_written && fmt->pb && !(fmt->oformat->flags & AVFMT_NOFILE)) {
        av_write_trailer(fmt);
      }
      if (fmt->pb) avio_closep(&fmt->pb);
      avformat_free_context(fmt);
      fmt = nullptr;
      stream = nullptr;
      header_written = false;
    }

    avcodec_free_context(&ctx);
    av_frame_free(&sw_frame);
    av_packet_free(&pkt);
    if (sws) {
      sws_freeContext(sws);
      sws = nullptr;
    }
    if (hw_device) {
      av_buffer_unref(&hw_device);
      hw_device = nullptr;
    }
  }

private:
  // 功能：打开 VAAPI 硬编编码器（h264_vaapi）
  // 输入：
  //   fps      标称帧率，Hz
  //   quality  质量 1 ~ 51，写入 global_quality
  // 输出：true 表示编码器已打开；false 表示无 h264_vaapi 或无可用 VAAPI 设备
  // 备注：失败时会残留部分资源，由调用方释放；B 帧强制关闭，保证帧序与输入一致
  bool open_hardware(int fps, int quality)
  {
    const AVCodec * codec = avcodec_find_encoder_by_name("h264_vaapi");
    if (!codec) return false;

    // 设备名先留空交给 libavutil 自选，失败再指定 Intel 的默认渲染节点
    if (av_hwdevice_ctx_create(&hw_device, AV_HWDEVICE_TYPE_VAAPI, nullptr, nullptr, 0) < 0) {
      if (
        av_hwdevice_ctx_create(
          &hw_device, AV_HWDEVICE_TYPE_VAAPI, "/dev/dri/renderD128", nullptr, 0) < 0) {
        return false;
      }
    }

    ctx = avcodec_alloc_context3(codec);
    if (!ctx) return false;
    ctx->width = width;
    ctx->height = height;
    ctx->time_base = AVRational{1, 1000};  // 毫秒时基，按真实采集时刻打时间戳
    ctx->framerate = AVRational{fps, 1};
    ctx->pix_fmt = AV_PIX_FMT_VAAPI;
    ctx->max_b_frames = 0;                 // 关 B 帧：延迟低，帧序与输入一致，便于复盘对号
    ctx->gop_size = std::max(1, fps);      // 每秒一个关键帧，拖动进度条不用从头解码
    ctx->global_quality = quality;         // 等价命令行的 -global_quality
    ctx->hw_device_ctx = av_buffer_ref(hw_device);

    // h264_vaapi 要求调用方在 avcodec_open2 之前提供硬件帧池（AVHWFramesContext）；
    // 只设 hw_device_ctx 会报 "A hardware frames reference is required"。
    // 命令行 ffmpeg 会自动补这一步，所以 CLI 测试能过，自己调 API 必须手写。
    AVBufferRef * hw_frames = av_hwframe_ctx_alloc(hw_device);
    if (!hw_frames) return false;
    auto * frames_ctx = reinterpret_cast<AVHWFramesContext *>(hw_frames->data);
    frames_ctx->format = AV_PIX_FMT_VAAPI;     // 对外的表面格式
    frames_ctx->sw_format = AV_PIX_FMT_NV12;   // 上传源格式，与 sws 输出一致
    frames_ctx->width = width;
    frames_ctx->height = height;
    if (av_hwframe_ctx_init(hw_frames) < 0) {
      av_buffer_unref(&hw_frames);
      return false;
    }
    ctx->hw_frames_ctx = hw_frames;  // 所有权交给 ctx，随 avcodec_free_context 释放

    // Matroska 把 SPS/PPS 放在文件头的 CodecPrivate 里，要求编码器在 open 阶段就给出
    // extradata。h264_vaapi 默认要等第一帧编完才生成，写头时是空的，封装器会报
    // "Invalid data found when processing input"。置这个标志让编码器立刻产出 extradata。
    ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    const int ret = avcodec_open2(ctx, codec, nullptr);
    if (ret < 0) {
      tools::logger()->warn("[Recorder] h264_vaapi 打开失败：{}", av_error_text(ret));
      return false;
    }

    hardware = true;
    sw_format = AV_PIX_FMT_NV12;
    return true;
  }

  // 功能：打开 libx264 软编编码器，作为硬件不可用时的兜底
  // 输入：
  //   fps      标称帧率，Hz
  //   quality  质量 1 ~ 51，直接作为 crf
  // 输出：true 表示编码器已打开；false 表示 libx264 不可用
  // 备注：preset=ultrafast + tune=zerolatency 是软编里最省 CPU 的组合，
  //       实测 1080p 约 8.5 ms/帧；仍比 MJPEG 的 14.8 ms/帧便宜
  bool open_software(int fps, int quality)
  {
    const AVCodec * codec = avcodec_find_encoder_by_name("libx264");
    if (!codec) return false;

    ctx = avcodec_alloc_context3(codec);
    if (!ctx) return false;
    ctx->width = width;
    ctx->height = height;
    ctx->time_base = AVRational{1, 1000};
    ctx->framerate = AVRational{fps, 1};
    ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    ctx->max_b_frames = 0;
    ctx->gop_size = std::max(1, fps);
    av_opt_set(ctx->priv_data, "preset", "ultrafast", 0);
    av_opt_set(ctx->priv_data, "tune", "zerolatency", 0);
    av_opt_set_int(ctx->priv_data, "crf", quality, 0);
    ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;  // 同上：Matroska 需要文件头里的 SPS/PPS

    const int ret = avcodec_open2(ctx, codec, nullptr);
    if (ret < 0) {
      tools::logger()->warn("[Recorder] libx264 打开失败：{}", av_error_text(ret));
      return false;
    }

    hardware = false;
    sw_format = AV_PIX_FMT_YUV420P;
    return true;
  }

  // 功能：创建 Matroska 封装器、软件帧与色彩转换上下文
  // 输入：
  //   path  输出文件路径
  // 输出：true 表示文件已打开且可写；false 表示创建或写文件头失败
  // 备注：容器选 Matroska 而不是 MP4/AVI：它不需要尾部索引，进程被强杀后文件仍可播
  bool open_muxer(const std::string & path)
  {
    int ret = avformat_alloc_output_context2(&fmt, nullptr, "matroska", path.c_str());
    if (ret < 0 || !fmt) {
      tools::logger()->error("[Recorder] 创建封装器失败：{}", av_error_text(ret));
      return false;
    }

    stream = avformat_new_stream(fmt, nullptr);
    if (!stream) {
      tools::logger()->error("[Recorder] 创建输出流失败");
      return false;
    }
    stream->time_base = ctx->time_base;
    ret = avcodec_parameters_from_context(stream->codecpar, ctx);
    if (ret < 0) {
      tools::logger()->error("[Recorder] 复制编码参数失败：{}", av_error_text(ret));
      return false;
    }

    if (!(fmt->oformat->flags & AVFMT_NOFILE)) {
      ret = avio_open(&fmt->pb, path.c_str(), AVIO_FLAG_WRITE);
      if (ret < 0) {
        tools::logger()->error("[Recorder] 打开 {} 失败：{}", path, av_error_text(ret));
        return false;
      }
    }
    ret = avformat_write_header(fmt, nullptr);
    if (ret < 0) {
      tools::logger()->error("[Recorder] 写文件头失败：{}", av_error_text(ret));
      return false;
    }
    header_written = true;  // 之后才允许 write_trailer

    sw_frame = av_frame_alloc();
    pkt = av_packet_alloc();
    if (!sw_frame || !pkt) {
      tools::logger()->error("[Recorder] 分配帧/包缓冲失败");
      return false;
    }
    sw_frame->format = sw_format;
    sw_frame->width = width;
    sw_frame->height = height;
    ret = av_frame_get_buffer(sw_frame, 32);
    if (ret < 0) {
      tools::logger()->error("[Recorder] 分配软件帧缓冲失败：{}", av_error_text(ret));
      return false;
    }

    // 宽高不变，只是色彩空间转换，用 SWS_POINT（最近邻）最快且不引入模糊
    sws = sws_getContext(
      width, height, AV_PIX_FMT_BGR24, width, height, sw_format, SWS_POINT, nullptr, nullptr,
      nullptr);
    return sws != nullptr;
  }

  // 功能：取走编码器已产出的全部码流包并写入容器
  // 输入：无
  // 输出：true 表示已排空且写盘正常；false 表示编码或写盘出错
  // 备注：EAGAIN 表示暂时没有新包（正常），EOF 表示编码器已关闭
  bool drain()
  {
    // 未写成文件头时禁止写包：muxer 处于半初始化状态，写包会踩空
    if (!ctx || !fmt || !stream || !header_written) return true;
    while (true) {
      const int ret = avcodec_receive_packet(ctx, pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) return true;
      if (ret < 0) return false;

      av_packet_rescale_ts(pkt, ctx->time_base, stream->time_base);
      pkt->stream_index = stream->index;
      const int write_ret = av_interleaved_write_frame(fmt, pkt);
      av_packet_unref(pkt);
      if (write_ret < 0) return false;
    }
  }
};

// 实现要点：目录与两个文件名在构造期一次算好（含启动时刻），运行期不再拼接字符串；
// 此处只建目录、不打开文件 —— 编码器需要首帧宽高，推迟到 init()。
Recorder::Recorder(double fps, const std::string & file_name, int quality)
: init_(false), stop_thread_(false), fps_(fps), quality_(quality), camera_name_(file_name), queue_(1)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;  // 首帧的限帧间隔从构造时刻算起

  // 文件名用系统时钟，只有它能区分重启前后的两次录制；时间轴仍统一用 steady_clock
  auto datetime = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto folder_path = "records/" + file_name;  // 相对进程工作目录
  std::filesystem::create_directories(folder_path);

  text_path_ = fmt::format("{}/{}.txt", folder_path, datetime);
  video_path_ = fmt::format("{}/{}.mkv", folder_path, datetime);
}

// 实现要点：先置退出标志再推空帧唤醒保存线程；join 之后才能关编码器，否则会与
// 正在编码的线程竞争 AVCodecContext。编码器的 flush 与文件尾都放在 impl_->close() 里。
Recorder::~Recorder()
{
  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免 pop 一直等待
  FrameData wakeup;  // img 为空：保存线程 wake 后直接 continue 并退出
  queue_.push(std::move(wakeup));
  if (saving_thread_.joinable()) saving_thread_.join();

  if (impl_) {
    impl_->close();
    impl_.reset();
  }
  if (text_writer_.is_open()) text_writer_.close();
}

// 实现要点：只有写盘线程降优先级（nice +10），不拖累自瞄主线程；设置失败只告警。
// 功能：把业务侧描述好的覆盖层画到图像上（保存线程内执行，紧邻编码之前）
// 输入：
//   img      目标图像，原地修改；与调用方共享像素，因此调用方在 record() 返回后不得再写它
//   overlay  要画的框、角点、标签与 HUD，纯数据
// 输出：无
// 备注：只对已经被限帧接受的帧调用；这里的 std::string 是保存线程内的小临时对象，不占主循环
void draw_video_overlay(cv::Mat & img, const VideoOverlay & overlay)
{
  const size_t count = std::min(overlay.armor_count, VideoOverlay::kMaxArmors);
  for (size_t i = 0; i < count; ++i) {
    const auto & armor = overlay.armors[i];
    cv::rectangle(img, armor.box, armor.color, 2);
    const std::string label(armor.label, armor.label_len);
    cv::putText(
      img, label, cv::Point(armor.box.x, armor.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      armor.color, 1);
    for (const auto & point : armor.points) {
      cv::circle(img, point, 2, cv::Scalar(0, 0, 255), -1);
    }
  }

  if (overlay.hud_len > 0) {
    const std::string hud(overlay.hud, overlay.hud_len);
    cv::putText(
      img, hud, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
  }
}

// 编码与写盘整段加 g_encode_mutex，实现全进程串行编码；文本写入在锁外，不占用编码窗口。
void Recorder::save_to_file()
{
  pid_t tid = static_cast<pid_t>(::syscall(SYS_gettid));  // 按线程调优先级需要 TID

  if (::setpriority(PRIO_PROCESS, tid, 10) != 0) {
    tools::logger()->warn(
      "[Recorder] Failed to lower saving thread priority: {}", std::strerror(errno));
  }

  // 编码失败计数：编码器一旦坏掉会每帧都失败，90fps 下不节流会把日志刷爆并拖慢主循环
  uint64_t fail_count = 0;

  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);  // 阻塞等待下一帧；析构推入的空帧用于唤醒本调用

    // 正常路径不会出现空图（record() 已过滤）；这里兜底，避免把 0 尺寸的图交给编码器
    if (frame.img.empty() || !impl_) continue;

    // 时间轴用相机采集时刻而不是 now：回放数据也能得到正确的相对时间
    const double since_begin = tools::delta_time(frame.timestamp, start_time_);
    const auto pts_ms = static_cast<int64_t>(std::llround(since_begin * 1000.0));

    // 埋点覆盖真实管线成本：覆盖层绘制 + 编码 + 写盘，全部在保存线程内
    frame_timer_.start();
    draw_video_overlay(frame.img, frame.overlay);

    bool encoded = false;
    {
      std::lock_guard<std::mutex> lock(g_encode_mutex);
      encoded = impl_->encode(frame.img, pts_ms);
    }
    frame_timer_.stop();

    if (!encoded) {
      // 只打印前 3 次，之后每 300 次打一条，既能看见故障又不会刷屏
      if (fail_count < 3 || fail_count % 300 == 0) {
        tools::logger()->warn(
          "[Recorder] {} 编码或写盘失败（累计 {} 帧），丢弃本帧", camera_name_, fail_count);
      }
      fail_count++;
      continue;
    }

    // Eigen 的 coeffs() 顺序为 (x, y, z, w)，这里重排成 txt 约定的 w x y z
    Eigen::Vector4d xyzw = frame.q.coeffs();
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
    text_writer_.flush();  // 每行落一次，进程崩溃时已录帧的时间戳不丢
  }
}

// 实现要点：三道过滤 —— 空图不录、首帧触发惰性 init()、按 fps_ 限帧。
// 限帧用 0.9 倍容忍：相机出帧周期本来就贴着 1/fps，严格比较会把 15fps 的流限成 7.5fps。
void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp, const VideoOverlay & overlay)
{
  if (img.empty()) return;
  if (!init_) init(img);
  if (!impl_) return;  // 编码器初始化失败，本次不录像

  // 三道过滤里只有这里做限帧：被丢弃的帧根本不会进入绘制与编码管线
  const double since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 0.9 / fps_) return;

  last_time_ = timestamp;
  // 图像浅拷贝、覆盖层按值入队（定长 POD，零堆分配）；队列容量 1，满则丢弃本帧
  queue_.push({img, q, timestamp, overlay});
}

// 实现要点：init_ 先置 true，保证编码器初始化失败也只尝试一次（重开 VAAPI 设备的代价
// 远高于放弃录像）。编码器与保存线程都建好之后才对外可见，避免半初始化状态。
void Recorder::init(const cv::Mat & img)
{
  init_ = true;

  auto impl = std::make_unique<Impl>();
  const int fps = std::max(1, static_cast<int>(std::lround(fps_)));
  if (!impl->open(video_path_, img.cols, img.rows, fps, quality_)) {
    tools::logger()->error("[Recorder] {} 录像初始化失败（原因见上方 error），本次不录像", camera_name_);
    return;
  }

  text_writer_.open(text_path_);
  const bool hardware = impl->hardware;
  impl_ = std::move(impl);
  saving_thread_ = std::thread(&Recorder::save_to_file, this);

  tools::logger()->info(
    "[Recorder] {} -> {} ({}x{} @{}fps, {})", camera_name_, video_path_, img.cols, img.rows, fps,
    hardware ? "h264_vaapi" : "libx264");
}

}  // namespace tools
