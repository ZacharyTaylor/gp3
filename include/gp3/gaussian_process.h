#ifndef GP3_GAUSSIAN_PROCESS_H
#define GP3_GAUSSIAN_PROCESS_H

#include <cmath>
#include <list>

#include <Eigen/Eigen>

#include <ros/ros.h>

class GaussianProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GaussianProcess(
      const double& lengthscale_sd, const double& signal_sd);

  double kernel(const double& a, const double& b,
                bool on_diagonal = false) const;

  void computeCovariance(const Eigen::VectorXd& time, const Eigen::VectorXd& data);

  double predict(const double& prediction_time);

  double predictDerivative(const double& prediction_time);

 private:
  const double lengthscale_var_;
  const double signal_var_;

  // needed for predicting values
  Eigen::VectorXd alpha_;
  Eigen::VectorXd time_;
};

#endif  // GP3_GAUSSIAN_PROCESS_H