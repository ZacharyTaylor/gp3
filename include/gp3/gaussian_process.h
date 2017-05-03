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

  double kernel(const ros::Time& a, const ros::Time& b,
                bool on_diagonal = false) const;

  void addDataPoint(const ros::Time& time, const double& value);

  void computeCovariance();

  double predict(const ros::Time& prediction_time);

  double predictDerivative(const ros::Time& prediction_time);

 private:

  struct Data{
    ros::Time time;
    double value;
  };

  const size_t max_data_points_;
  const double lengthscale_var_;
  const double signal_var_;

  //stored data
  std::list<Data> data_list_;

  // needed for predicting values
  Eigen::VectorXd alpha_;
};

#endif  // GP3_GAUSSIAN_PROCESS_H