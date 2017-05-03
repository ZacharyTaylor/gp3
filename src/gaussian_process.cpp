#include "gp3/gaussian_process.h"

GaussianProcess::GaussianProcess(const double& lengthscale_sd,
                                 const double& signal_sd)
    : lengthscale_var_(lengthscale_sd * lengthscale_sd),
      signal_var_(signal_sd * signal_sd){};

double GaussianProcess::kernel(const double& a, const double& b,
                               bool on_diagonal) const {
  double sq_dist = (a - b) * (a - b);
  double error = std::exp(-sq_dist / (2.0 * lengthscale_var_));

  if (on_diagonal) {
    error += signal_var_;
  }

  return error;
}

void GaussianProcess::computeCovariance(const Eigen::VectorXd& time,
                                        const Eigen::VectorXd& data) {
  if (time.size() != data.size()) {
    throw std::runtime_error(
        "time and data must contain same number of points");
  }

  Eigen::MatrixXd K;
  K.resize(data.size(), data.size());

  // loop over computing elements of the covariance using the specified
  // kernel. Only lower triangular elements are computed as the others won't
  // be used.
  for (size_t i = 0; i < time.size(); ++i) {
    for (size_t j = 0; j <= i; ++j) {
      K(i, j) = kernel(time[i], time[j], i == j);
    }
  }

  // calculate alpha (K-1*y)
  alpha_ = K.llt().solve(data);
  time_ = time;
}

// get the prediction at the specified time
double GaussianProcess::predict(const double& prediction_time) {
  if (alpha_.size() == 0) {
    throw std::runtime_error("Prediction requested before system was ready");
  }

  double predicted_value = 0;
  for (size_t i = 0; i < alpha_.size(); ++i) {
    predicted_value += kernel(time_[i], prediction_time) * alpha_[i];
  }
  return predicted_value;
}

// get the derivative of the prediction at the specified time
double GaussianProcess::predictDerivative(const double& prediction_time) {
  if (alpha_.size() == 0) {
    throw std::runtime_error("Prediction requested before system was ready");
  }

  double predicted_derivative = 0;
  for (size_t i = 0; i < alpha_.size(); ++i) {
    predicted_derivative += (prediction_time - time_[i]) *
                            kernel(time_[i], prediction_time) * alpha_[i];
  }

  predicted_derivative /= -lengthscale_var_;
  return predicted_derivative;
}