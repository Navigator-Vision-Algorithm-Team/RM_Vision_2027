#ifndef OMNIPERCEPTION__OMNI_SELECTION_HPP
#define OMNIPERCEPTION__OMNI_SELECTION_HPP

/*
 * 全向相机启用集合的解析：把配置里的 omni_camera_ids / omni_camera_count
 * 换算成一组物理编号，供各入口的初始化循环使用。
 *
 * 数据流：配置节点 → 本函数 → 升序去重的编号列表 → 入口按编号创建相机与任务。
 *
 * 依赖与边界：只读配置，不创建相机、不打开设备；编号上限由本文件集中定义，
 *             入口侧不再各自写死 4。
 *
 * 使用注意：
 *   * 新键 omni_camera_ids 存在时优先，即使同时写了 omni_camera_count；
 *   * omni_camera_ids 写成 [] 表示「一个都不启用」，与「键缺省」是两件事；
 *   * 编号重复自动去重、乱序自动升序，保证每次启动的初始化顺序一致；
 *   * 越界、非整数、类型不是列表都抛异常——少开一路相机却没有提示是现场最难查的故障。
 */

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "tools/logger.hpp"

namespace omniperception
{

// 支持的全向相机最大编号；新增相机时只改这一处
inline constexpr int kMaxOmniCameras = 4;

// 功能：解析本次要启用的全向相机编号
// 输入：
//   yaml  配置根节点；读取 omni_camera_ids（优先）与 omni_camera_count（兼容旧配置）
// 输出：升序去重的编号列表，元素取值 1~kMaxOmniCameras；空列表表示不启用全向
// 备注：解析结果的最终集合会打印一次 info，便于现场核对"到底开了哪几路"
inline std::vector<int> resolve_omni_camera_ids(const YAML::Node & yaml)
{
  std::vector<int> ids;

  const auto ids_node = yaml["omni_camera_ids"];
  if (ids_node) {
    if (!ids_node.IsSequence()) {
      throw std::runtime_error(
        "omni_camera_ids 必须是列表，例如 [1, 3]；当前写成了标量或其他类型");
    }

    for (const auto & item : ids_node) {
      int id = 0;
      try {
        id = item.as<int>();
      } catch (const std::exception &) {
        throw std::runtime_error("omni_camera_ids 中含有非整数元素，只接受 1~" +
                                 std::to_string(kMaxOmniCameras) + " 的编号");
      }
      if (id < 1 || id > kMaxOmniCameras) {
        throw std::runtime_error(
          "omni_camera_ids 中的编号 " + std::to_string(id) + " 超出范围，只接受 1~" +
          std::to_string(kMaxOmniCameras));
      }
      ids.push_back(id);
    }

    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());

    if (yaml["omni_camera_count"]) {
      tools::logger()->warn(
        "[omni] 同时配置了 omni_camera_ids 与 omni_camera_count，以 omni_camera_ids 为准，后者被忽略");
    }
  } else {
    // 旧配置：按数量 1..N 递增启用
    const int count = yaml["omni_camera_count"] ? yaml["omni_camera_count"].as<int>() : 0;
    if (count < 0 || count > kMaxOmniCameras) {
      throw std::runtime_error(
        "omni_camera_count 超出范围，只接受 0~" + std::to_string(kMaxOmniCameras));
    }
    for (int i = 1; i <= count; ++i) ids.push_back(i);
  }

  if (ids.empty()) {
    tools::logger()->info("[omni] 未启用全向相机（仅主相机）");
  } else {
    std::string text;
    for (int id : ids) {
      if (!text.empty()) text += ", ";
      text += "omni" + std::to_string(id);
    }
    tools::logger()->info("[omni] 启用全向相机：{}", text);
  }

  return ids;
}

}  // namespace omniperception

#endif  // OMNIPERCEPTION__OMNI_SELECTION_HPP
