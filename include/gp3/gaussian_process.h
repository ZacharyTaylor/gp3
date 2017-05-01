#ifndef GP3_GAUSSIAN_PROCESS_H
#define GP3_GAUSSIAN_PROCESS_H

#include <cmath>
#include <list>

#include <Eigen/Eigen>

#include <ros/ros.h>

typedef std::pair<ros::Time, double> Data;

class Kernel {
 public:
  Kernel(const double& lengthscale_sd, const double& signal_sd);

  virtual double operator()(const double& a, const double& b) const;

 protected:
  double lengthscale_var_;
  double signal_var_;
};

class SqExpKernel : public Kernel {
 public:
  SqExpKernel(const double& lengthscale_sd, const double& signal_sd);
  double operator()(const double& a, const double& b) const;
};

class SqExpAngleWrapKernel : public Kernel {
 public:
  SqExpAngleWrapKernel(const double& lengthscale_sd, const double& signal_sd);
  double operator()(const double& a, const double& b) const;
};

class GaussianProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GaussianProcess(
      const Kernel& kernel, const size_t& max_data_points,
      const double& time_diff_warning = std::numeric_limits<double>::max());

  void addDataPoint(const ros::Time& time, const double& data_point,
                    const bool recompute_cov = true);

  void computeCovariance();

  // get the prediction at the specified time
  void predict(const ros::Time& prediction_time, double* predicted_value,
               double* prediction_sd = nullptr);

 private:
  const Kernel kernel_;
  const size_t max_data_points_;
  const double time_diff_warning_;

  std::list<Data> data_list_;
  std::vector<double> time_diffs_;

  // covariance
  Eigen::LLT<Eigen::MatrixXd> L_llt_;
  bool ready_to_predict_;

  // needed for predicting values
  Eigen::VectorXd mu_;
};

#endif  // GP3_GAUSSIAN_PROCESS_H