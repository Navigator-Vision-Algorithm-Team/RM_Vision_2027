#include "pnp.hpp"

namespace auto_aim
{

// =============================================================================
//  装甲板尺寸常量
//  注意：这三个常量原来定义在 solver.cpp 中，现在移到这里，由主相机和全向相机共用。
//        数值保持与原来完全一致，避免改变主相机的解算结果。
// =============================================================================
const double LIGHTBAR_LENGTH = 56e-3;     // m，灯条长度（装甲板高度）
const double BIG_ARMOR_WIDTH = 230e-3;    // m，大装甲板宽度
const double SMALL_ARMOR_WIDTH = 135e-3;  // m，小装甲板宽度

// 大装甲板的 4 个模型点（装甲板自身坐标系：x 朝外，y 沿板宽，z 沿灯条）
static const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

// 小装甲板的 4 个模型点（顺序与上面一致）
static const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

const std::vector<cv::Point3f> & armor_object_points(ArmorType type)
{
  return (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
}

PnPResult solve_pnp_armor(
  const Armor & armor, const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs)
{
  PnPResult result;

  // 必须有 4 个像素点（YOLO 输出的装甲板 4 个角点）才能做平面 IPPE。
  if (armor.points.size() != 4 || camera_matrix.empty()) return result;

  const auto & object_points = armor_object_points(armor.type);

  // 畸变系数允许为空：solvePnP 接受空 Mat 表示无畸变。
  cv::Vec3d rvec, tvec;
  bool ok = cv::solvePnP(
    object_points, armor.points, camera_matrix, distort_coeffs, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
  if (!ok) return result;

  // tvec：装甲板中心在相机系下的位置
  cv::cv2eigen(tvec, result.xyz_in_camera);

  // rvec：装甲板 -> 相机的旋转
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  cv::cv2eigen(rmat, result.R_armor2camera);

  // 重投影误差：把 4 个模型点按解出来的位姿投回图像，与检测点比较。
  // 全向相机距离远、遮挡多，这个值可以用来过滤掉明显错误的解。
  std::vector<cv::Point2f> reprojected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, distort_coeffs, reprojected);
  double sum = 0.0;
  for (std::size_t i = 0; i < reprojected.size(); ++i) {
    sum += cv::norm(armor.points[i] - reprojected[i]);
  }
  result.reprojection_error = sum / static_cast<double>(reprojected.size());

  result.success = true;
  return result;
}

}  // namespace auto_aim
