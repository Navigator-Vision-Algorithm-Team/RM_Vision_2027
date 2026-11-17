#ifndef TOOLS__VIDEO_OVERLAY_HPP
#define TOOLS__VIDEO_OVERLAY_HPP

/*
 * 录像画面的覆盖层描述：只保存"要画什么"的纯数据，不含业务逻辑。
 *
 * 数据流：业务侧把检测结果填成 VideoOverlay → Recorder::record() 随帧入队
 *         → 保存线程在编码之前按这份描述画到图像上。
 *
 * 依赖与边界：只依赖 OpenCV 的基础几何类型；**不依赖 Armor / Tracker / 业务枚举**，
 *             颜色与文字在业务侧折算成标量与字符串后传入。
 *
 * 设计取舍：用定长数组而非 std::vector/std::string，保证入队零堆分配；
 *           上限取 16 块装甲板、64 字节标签、256 字节 HUD，超出部分截断而非扩容。
 *
 * 使用注意：本结构只描述像素覆盖内容；PnP 结果、IMM 指标等纯调试数据不进这里，
 *           仍由各入口的 DebugStream 单独发布。
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include <opencv2/core.hpp>

namespace tools
{

// 单块装甲板要画的内容
struct OverlayArmor
{
  cv::Rect box{};                 // 框，像素坐标
  cv::Point2f points[4]{};        // 四个角点，像素坐标
  cv::Scalar color{};             // 框与文字颜色，BGR
  char label[64]{};               // 框上方文字，如 "0.95 RED small"
  uint16_t label_len = 0;         // 实际字符数，不含结尾 '\0'
};

// 一帧画面的全部覆盖层
struct VideoOverlay
{
  static constexpr size_t kMaxArmors = 16;  // 单帧最多覆盖的装甲板数

  OverlayArmor armors[kMaxArmors]{};  // 装甲板列表，只用前 armor_count 项
  size_t armor_count = 0;             // 有效装甲板数，超过上限时截断

  char hud[256]{};       // 左上角状态行
  uint16_t hud_len = 0;  // 实际字符数，不含结尾 '\0'
};

// 功能：把文本安全写入覆盖层的定长字符数组
// 输入：
//   dst  目标数组
//   len  实际长度输出参数
//   cap  dst 的容量（含结尾 '\0'）
//   text 源文本
// 输出：写入 dst 并更新 len，超长部分截断
// 备注：只做一次 memcpy，无堆分配；由业务侧在组装覆盖层时调用
inline void set_overlay_text(char * dst, uint16_t & len, size_t cap, const std::string & text)
{
  const size_t n = std::min(text.size(), cap - 1);
  std::memcpy(dst, text.data(), n);
  dst[n] = '\0';
  len = static_cast<uint16_t>(n);
}

}  // namespace tools

#endif  // TOOLS__VIDEO_OVERLAY_HPP
