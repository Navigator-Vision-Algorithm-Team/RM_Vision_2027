#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_aim/pnp.hpp"
#include "tasks/auto_aim/solver.hpp"

namespace
{
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}

void check(const std::string & config_path)
{
  const auto yaml = YAML::LoadFile(config_path);
  const auto k = yaml["camera_matrix"].as<std::vector<double>>();
  const auto d = yaml["distort_coeffs"].as<std::vector<double>>();
  const cv::Mat K = cv::Mat(3, 3, CV_64F, const_cast<double *>(k.data())).clone();
  const cv::Mat D = cv::Mat(1, 5, CV_64F, const_cast<double *>(d.data())).clone();
  const Eigen::Matrix3d R = (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();
  const Eigen::Vector3d t(0, 0.07764, 0.005);
  const auto actual_r = yaml["R_camera2gimbal"].as<std::vector<double>>();
  const auto actual_t = yaml["t_camera2gimbal"].as<std::vector<double>>();
  require(actual_r.size() == 9 && actual_t.size() == 3, "extrinsic dimensions");
  require(Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(actual_r.data()).isApprox(R), "parallel mount rotation");
  require(Eigen::Vector3d(actual_t.data()).isApprox(t), "mount translation in metres");
  require(yaml["yaw_offset"].as<double>() == 0.0 && yaml["pitch_offset"].as<double>() == 0.0,
          "mount translation must not also be configured as an angle offset");
  auto_aim::Solver solver(config_path);

  // Use undistortion as an independent check of the expected right/down parallax.
  for (double depth : {1.0, 3.0, 10.0}) {
    const auto pixel = solver.gimbal_axis_pixel(depth);
    require(pixel.has_value(), "forward axis must project");
    std::vector<cv::Point2f> normalized;
    cv::undistortPoints(std::vector<cv::Point2f>{*pixel}, normalized, K, D);
    require(std::abs(normalized[0].x - 0.07764 / depth) < 1e-6, "horizontal parallax sign/scale");
    require(std::abs(normalized[0].y - 0.005 / depth) < 1e-6, "vertical parallax sign/scale");

    // Axis projection must agree with world projection at a nonzero yaw/pitch/roll.
    const Eigen::Matrix3d pose =
      (Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX())).toRotationMatrix();
    solver.set_R_gimbal2world(pose);
    const Eigen::Vector3d world = pose * Eigen::Vector3d(depth, 0, 0);
    const auto projected = solver.world2pixel({cv::Point3f(world.x(), world.y(), world.z())});
    require(projected.size() == 1 && cv::norm(projected[0] - *pixel) < 1e-3, "world/axis projection consistency");
  }
  for (double depth : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    require(!solver.gimbal_axis_pixel(depth), "invalid depth must not project");
  }

  // Synthetic armor on the CAMERA optical axis: the execution frame must see it
  // to the left and above, so Aimer receives a nonzero yaw and height.
  solver.set_R_gimbal2world(Eigen::Matrix3d::Identity().eval());
  const Eigen::Matrix3d armor_to_camera = R.transpose() *
    (Eigen::AngleAxisd(3.0, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY())).toRotationMatrix();
  cv::Mat rotation, rvec;
  cv::eigen2cv(armor_to_camera, rotation);
  cv::Rodrigues(rotation, rvec);
  std::vector<cv::Point2f> corners;
  cv::projectPoints(auto_aim::armor_object_points(auto_aim::big), rvec,
                    cv::Vec3d(0, 0, 3), K, D, corners);
  auto_aim::Armor armor(0, 1.0f, cv::Rect(), corners);
  armor.type = auto_aim::big;
  armor.name = auto_aim::three;  // Skip the optional armor-yaw optimizer.
  require(solver.solve(armor), "synthetic PnP must succeed");
  require((armor.xyz_in_gimbal - Eigen::Vector3d(3, 0.07764, 0.005)).norm() < 1e-3,
          "PnP must include mount translation exactly once");
  require(armor.ypd_in_world.x() > 0 && armor.xyz_in_world.z() > 0, "aim correction direction");

  // Reciprocal check: a target AT the reticle must be on the execution axis.
  // Either a reversed sign or a second translation would leave a nonzero y/z.
  const Eigen::Vector3d camera_axis_target = R.transpose() * (Eigen::Vector3d(3, 0, 0) - t);
  cv::projectPoints(auto_aim::armor_object_points(auto_aim::big), rvec,
                    cv::Vec3d(camera_axis_target.x(), camera_axis_target.y(), camera_axis_target.z()),
                    K, D, armor.points);
  require(solver.solve(armor), "execution-axis synthetic PnP must succeed");
  require((armor.xyz_in_gimbal - Eigen::Vector3d(3, 0, 0)).norm() < 1e-3,
          "reticle target must cancel mount translation exactly once");
  require(std::abs(armor.ypd_in_world.x()) < 1e-4 && std::abs(armor.xyz_in_world.z()) < 1e-4,
          "reticle target must require no geometric yaw/height correction");

  // The actual Solver pose used by DebugStream::publish must recover forward
  // depth, not slant range; off-axis targets make these quantities different.
  const Eigen::Matrix3d pose =
    (Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY())).toRotationMatrix();
  solver.set_R_gimbal2world(pose);
  const Eigen::Vector3d aim_world = pose * Eigen::Vector3d(3, 1, 0.5);
  const double depth = (solver.R_gimbal2world().transpose() * aim_world).x();
  require(std::abs(depth - 3) < 1e-12, "reticle depth must use the active Solver pose");
  require(cv::norm(*solver.gimbal_axis_pixel(depth) - *solver.gimbal_axis_pixel(3)) < 1e-3,
          "reticle must match active Solver at the recovered forward depth");

  solver.set_extrinsic(R, Eigen::Vector3d::Zero());
  const auto principal = solver.gimbal_axis_pixel(3);
  require(principal && cv::norm(*principal - cv::Point2f(k[2], k[5])) < 1e-3, "zero offset principal point");
  solver.set_extrinsic(R, Eigen::Vector3d(4, 0, 0));
  require(!solver.gimbal_axis_pixel(3), "point behind camera must not project");
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "usage: camera_geometry_test config.yaml");
    check(argv[1]);
    std::cout << "PASS: mount geometry, PnP and reticle projection\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
