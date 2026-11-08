#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

struct Lightbar
{
  // 全部给默认值：Armor 里有两个 Lightbar 成员，走神经网络构造函数时它们是默认构造的，
  // 残留的未初始化值一旦被读到就是未定义行为。
  std::size_t id = 0;
  Color color = Color::blue;
  cv::Point2f center{0, 0}, top{0, 0}, bottom{0, 0}, top2bottom{0, 0};
  std::vector<cv::Point2f> points;
  double angle = 0.0, angle_error = 0.0, length = 0.0, width = 0.0, ratio = 0.0;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() = default;
};

struct Armor
{
  Color color = Color::blue;
  Lightbar left, right;     //used to be const
  cv::Point2f center{0, 0};       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm{0, 0};  // 归一化坐标
  std::vector<cv::Point2f> points;

  double ratio = 0.0;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio = 0.0;         // 长灯条与短灯条的长度之比
  double rectangular_error = 0.0;  // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type = ArmorType::small;
  ArmorName name = ArmorName::not_armor;
  
  ArmorPriority priority = ArmorPriority::fifth;
  int class_id = 0;
  cv::Rect box;
  cv::Mat pattern;
  double confidence = 0.0;
  bool duplicated = false;

  // 【改动】给出确定的初值：Solver::solve / 全向 PnP 在解算失败时会提前返回，
  // 若这些成员未初始化，下游 tracker 会读到不确定的值。默认置 0 更安全。
  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();  // 单位：m
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();   // 单位：m
  Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero();  // 单位：rad
  Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();   // 单位：rad
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();   // 球坐标系

  double yaw_raw = 0.0;  // rad

  Armor(const Lightbar & left, const Lightbar & right);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP