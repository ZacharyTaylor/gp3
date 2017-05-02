#ifndef GP3_GAUSSIAN_PROCESS_H
#define GP3_GAUSSIAN_PROCESS_H

#include <cmath>
#include <list>

#include <Eigen/Eigen>

#include <ros/ros.h>

typedef std::pair<ros::Time, double> Data;

class GaussianProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GaussianProcess(
      const double& lengthscale_sd, const double& signal_sd,
      const size_t& max_data_points, const bool& is_angle,
      const double& time_diff_warning = std::numeric_limits<double>::max());

  double kernel(const double& a, const double& b,
                bool on_diagonal = false) const;

  void addDataPoint(const ros::Time& time, const double& data_point,
                    const bool recompute_cov = true);

  void computeCovariance();

  // get the prediction at the specified time
  double predict(const ros::Time& prediction_time);

 private:
  const double lengthscale_var_;
  const double signal_var_;
  const size_t max_data_points_;
  const bool is_angle_;
  const double time_diff_warning_;

  std::list<Data> data_list_;
  std::vector<double> time_diffs_;

  // covariance
  Eigen::MatrixXd L_;
  bool ready_to_predict_;

  // needed for predicting values
  Eigen::VectorXd mu_;
};

#endif  // GP3_GAUSSIAN_PROCESS_H