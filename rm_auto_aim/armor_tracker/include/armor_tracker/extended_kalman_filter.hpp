// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;

  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  // State addition function:
  // x_new = x_add(x, dx)
  // Default behavior: x + dx
  using StateAddFunc = std::function<Eigen::VectorXd(
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
  )>;

  // Measurement residual function:
  // residual = z_subtract(z, z_pred)
  // Default behavior: z - z_pred
  using MeasureSubtractFunc = std::function<Eigen::VectorXd(
    const Eigen::VectorXd &, 
    const Eigen::VectorXd &
  )>;

  explicit ExtendedKalmanFilter(
    const VecVecFunc & f, 
    const VecVecFunc & h, 
    const VecMatFunc & j_f, 
    const VecMatFunc & j_h,
    const VoidMatFunc & u_q, 
    const VecMatFunc & u_r, 
    const Eigen::MatrixXd & P0,
    const StateAddFunc & x_add = defaultStateAdd,
    const MeasureSubtractFunc & z_subtract = defaultMeasureSubtract
  );

  void setState(const Eigen::VectorXd & x0);// Set the initial state
  Eigen::MatrixXd predict();// Compute a predicted state
  Eigen::MatrixXd update(const Eigen::VectorXd & z);// Update the estimated state based on measurement

  double getNIS() const { return nis; }

private:
  
  VecVecFunc f;// Process nonlinear vector function
  VecVecFunc h;// Observation nonlinear vector function
  VecMatFunc jacobian_f;// Jacobian of f()
  VecMatFunc jacobian_h;// Jacobian of h()
  VoidMatFunc update_Q;// Process noise covariance matrix
  VecMatFunc update_R;// Measurement noise covariance matrix
  
  Eigen::MatrixXd F;// Priori error estimate covariance matrix
  Eigen::MatrixXd H;// Posteriori error estimate covariance matrix
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd P_pri;
  Eigen::MatrixXd P_post;
  Eigen::MatrixXd K;
  Eigen::MatrixXd I;
  Eigen::VectorXd x_pri;
  Eigen::VectorXd x_post;

  StateAddFunc x_add;
  MeasureSubtractFunc z_subtract;
  
  int n;
  double nis = 0.0;
  static Eigen::VectorXd defaultStateAdd(const Eigen::VectorXd & x, const Eigen::VectorXd & dx){
    return x + dx;
  }

  static Eigen::VectorXd defaultMeasureSubtract(const Eigen::VectorXd & z, const Eigen::VectorXd & z_pred){
    return z - z_pred;
  }
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
