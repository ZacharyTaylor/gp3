#include "gp3/gaussian_process.h"

GaussianProcess::GaussianProcess(const size_t& max_data_points,
                                 const double& lengthscale_sd,
                                 const double& signal_sd)
    : max_data_points_(max_data_points),
      lengthscale_var_(lengthscale_sd * lengthscale_sd),
      signal_var_(signal_sd * signal_sd){};

double GaussianProcess::kernel(const ros::Time& a, const ros::Time& b,
                               bool on_diagonal) const {
  double diff = (a - b).toSec();
  double error = std::exp(-(diff * diff) / (2.0 * lengthscale_var_));

  if (on_diagonal) {
    error += signal_var_;
  }

  return error;
}

void GaussianProcess::addDataPoint(const ros::Time& time, const double& value) {
  Data data;
  data.time = time;
  data.value = value;
  data_list_.push_back(data);

  if (data_list_.size() > max_data_points_) {
    data_list_.pop_front();
  }

  cov_ready_ = false;
}

void GaussianProcess::computeCovariance() {
  if (data_list_.size() == 0) {
    throw std::runtime_error("Prediction requested before system was ready");
  }

  Eigen::MatrixXd K;
  Eigen::VectorXd data;
  K.resize(data_list_.size(), data_list_.size());
  data.resize(data_list_.size());

  // loop over computing elements of the covariance using the specified
  // kernel. Only lower triangular elements are computed as the others won't
  // be used.
  std::list<Data>::const_iterator it_i = data_list_.begin();
  for (size_t i = 0; i < data_list_.size(); ++i) {
    std::list<Data>::const_iterator it_j = data_list_.begin();
    for (size_t j = 0; j <= i; ++j) {
      K(i, j) = kernel(it_i->time, it_j->time, i == j);
      ++it_j;
    }

    // build data vector
    data(i) = it_i->value;
    ++it_i;
  }

  K_llt_ = K.llt();

  // calculate alpha (K-1*y)
  alpha_ = K_llt_.solve(data);
}

// get the prediction at the specified time
double GaussianProcess::predict(const ros::Time& prediction_time) {
  if (!cov_ready_) {
    computeCovariance();
  }

  double predicted_value = 0;
  std::list<Data>::const_iterator it = data_list_.begin();
  for (size_t i = 0; i < alpha_.size(); ++i) {
    predicted_value += kernel(it->time, prediction_time) * alpha_[i];
    ++it;
  }
  return predicted_value;
}

// get the uncertainty of the prediction at the specified time
double GaussianProcess::predictVariance(const ros::Time& prediction_time) {
  if (!cov_ready_) {
    computeCovariance();
  }

  Eigen::VectorXd K_s;
  K_s.resize(data_list_.size());
  std::list<Data>::const_iterator it = data_list_.begin();
  for (size_t i = 0; i < alpha_.size(); ++i) {
    K_s(i) = kernel(it->time, prediction_time);
    ++it;
  }

  Eigen::MatrixXd var = K_s.transpose() * K_llt_.solve(K_s);
  return kernel(prediction_time, prediction_time, true) - var(0, 0);
}

// get the derivative of the prediction at the specified time
double GaussianProcess::predictDerivative(const ros::Time& prediction_time) {
  if (!cov_ready_) {
    computeCovariance();
  }

  double predicted_derivative = 0;
  std::list<Data>::const_iterator it = data_list_.begin();
  for (size_t i = 0; i < alpha_.size(); ++i) {
    predicted_derivative += (prediction_time - it->time).toSec() *
                            kernel(it->time, prediction_time) * alpha_[i];
    ++it;
  }

  predicted_derivative /= -lengthscale_var_;
  return predicted_derivative;
}