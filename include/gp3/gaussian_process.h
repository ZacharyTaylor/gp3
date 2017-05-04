#ifndef GP3_GAUSSIAN_PROCESS_H
#define GP3_GAUSSIAN_PROCESS_H

#include <cmath>
#include <list>

#include <Eigen/Eigen>

#include <ros/ros.h>

class GaussianProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GaussianProcess(const size_t& max_data_points,
      const double& lengthscale_sd, const double& signal_sd);

  void addDataPoint(const ros::Time& time, const double& value);

  double predict(const ros::Time& prediction_time);

  double predictVariance(const ros::Time& prediction_time);

  double predictDerivative(const ros::Time& prediction_time);

 private:

  struct Data{
    ros::Time time;
    double value;
  };

  double kernel(const ros::Time& a, const ros::Time& b,
                bool on_diagonal = false) const;

  void computeCovariance();

  const size_t max_data_points_;
  const double lengthscale_var_;
  const double signal_var_;

  bool cov_ready_;

  //stored data
  std::list<Data> data_list_;

  // needed for predicting values
  Eigen::VectorXd alpha_;
  Eigen::LLT<Eigen::MatrixXd> K_llt_;
};

#endif  // GP3_GAUSSIAN_PROCESS_H