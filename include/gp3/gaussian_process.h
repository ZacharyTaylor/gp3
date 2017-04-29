#ifndef GP3_GAUSSIAN_PROCESS_H
#define GP3_GAUSSIAN_PROCESS_H

#include <cmath>
#include <list>

#include <Eigen/Eigen>

#include <ros/ros.h>

typedef std::pair<ros::Time, double> Data;

class Kernel {
  Kernel(const double& lengthscale_sd, const double& signal_sd)
      : lengthscale_var_(lengthscale_sd * lengthscale_sd),
        signal_var_(signal_sd * signal_sd){};

 protected:
  double lengthscale_var_;
  double signal_var_;

 public:
  virtual double operator()(const double& a, const double& b) const;
};

class SqExpKernel : public Kernel {
 public:
  double operator()(const double& a, const double& b) const {
    double sq_dist = (a - b) * (a - b);
    return signal_var_ * std::exp(-sq_dist / (2 * lengthscale_var_));
  }
};

// solves wrap around issue for angles
class SqExpAngleWrapKernel : public Kernel {
 public:
  double operator()(const double& a, const double& b) const {
    double abs_dist = std::abs(a - b);
    abs_dist = std::min(abs_dist, std::abs(abs_dist - 2 * M_PI));
    double sq_dist = abs_dist * abs_dist;
    return signal_var_ * std::exp(-sq_dist / (2 * lengthscale_var_));
  }
};

class GaussianProcess {
 public:
  GaussianProcess(const Kernel& kernel, const size_t& max_data_points)
      : kernel_(kernel),
        max_data_points_(max_data_points),
        ready_to_predict_(false){};

  void addDataPoint(const ros::Time& time, const double& data_point,
                    const bool recompute_cov = true) {
    data_list_.emplace_back(time, data_point);

    if (data_list_.size() > max_data_points_) {
      data_list_.pop_front();
    }
    ready_to_predict_ = false;

    if (recompute_cov) {
      computeCovariance();
    }
  }

  void computeCovariance() {
    if (data_list_.size() == 0) {
      throw std::runtime_error(
          "Covariance requested before system had any data");
    }

    Eigen::MatrixXd K;
    K.resize(data_list_.size(), data_list_.size());

    // convert times into times since first reading in list
    time_diffs_.resize(data_list_.size());
    std::vector<double>::iterator time_it = time_diffs_.begin();
    for (const Data& data : data_list_) {
      *time_it = (data.first - data_list_.front().first).toSec();
      ++time_it;
    }

    // loop over computing each element. Only lower triangular elements are
    // computed as the others won't be used.
    for (size_t i = 0; i < data_list_.size(); ++i) {
      for (size_t j = 0; j <= i; ++j) {
        K(i, j) = kernel_(time_diffs_[i], time_diffs_[j]);
      }
    }

    // use cholesky decomposition and get llt of lower triangular matrix
    L_llt_ = Eigen::MatrixXd(K.llt().matrixL()).llt();

    // calculate mu
    Eigen::MatrixXd data_values;
    data_values.resize(data_list_.size(), 1);

    std::list<Data>::iterator data_it = data_list_.begin();
    for (size_t i = 0; i < data_list_.size(); ++i) {
      data_values(i, 1) = data_it->second;
      ++data_it;
    }
    mu_ = L_llt_.solve(data_values);

    ready_to_predict_ = true;
  }

  // get the prediction at the specified time
  void predict(const ros::Time& prediction_time, double* predicted_value,
               double* prediction_sd) {
    if (!ready_to_predict_) {
      throw std::runtime_error("Prediction requested before system was ready");
    }

    double prediction_time_diff =
        (prediction_time - data_list_.front().first).toSec();

    Eigen::MatrixXd K_s;
    K_s.resize(1, data_list_.size());
    for (size_t i = 0; i < data_list_.size(); ++i) {
      K_s(1, i) = kernel_(time_diffs_[i], prediction_time_diff);
    }

    Eigen::MatrixXd L_k = L_llt_.solve(K_s);

    *predicted_value = 0;
    for (size_t i = 0; i < data_list_.size(); ++i) {
      *predicted_value += L_k(i, 1) + mu_(i, 1);
    }

    *prediction_sd = kernel_(prediction_time_diff, prediction_time_diff);
    for (size_t i = 0; i < data_list_.size(); ++i) {
      *prediction_sd -= L_k(i, 1) * L_k(i, 1);
    }
    *prediction_sd = std::sqrt(*prediction_sd);
  }

 private:
  const Kernel kernel_;
  const size_t max_data_points_;

  std::list<Data> data_list_;
  std::vector<double> time_diffs_;

  // covariance
  Eigen::LLT<Eigen::MatrixXd> L_llt_;
  bool ready_to_predict_;

  // needed for predicting values
  Eigen::MatrixXd mu_;
};

#endif  // GP3_GAUSSIAN_PROCESS_H