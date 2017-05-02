#include "gp3/gaussian_process.h"

GaussianProcess::GaussianProcess(const double& lengthscale_sd,
                                 const double& signal_sd,
                                 const size_t& max_data_points,
                                 const bool& is_angle,
                                 const double& time_diff_warning)
    : lengthscale_var_(lengthscale_sd * lengthscale_sd),
      signal_var_(signal_sd * signal_sd),
      max_data_points_(max_data_points),
      is_angle_(is_angle),
      time_diff_warning_(time_diff_warning),
      ready_to_predict_(false){};

double GaussianProcess::kernel(const double& a, const double& b,
                               bool on_diagonal) const {
  double sq_dist = (a - b) * (a - b);
  double error = std::exp(-sq_dist / (2.0 * lengthscale_var_));

  if (on_diagonal) {
    error += signal_var_;
  }

  return error;
}

void GaussianProcess::addDataPoint(const ros::Time& time,
                                   const double& data_point,
                                   const bool recompute_cov) {
  data_list_.emplace_back(time, data_point);

  while (data_list_.size() > max_data_points_) {
    data_list_.pop_front();
  }
  ready_to_predict_ = false;

  if (recompute_cov) {
    computeCovariance();
  }
}

void GaussianProcess::computeCovariance() {
  if (data_list_.size() == 0) {
    throw std::runtime_error("Covariance requested before system had any data");
  }

  Eigen::MatrixXd K;
  K.resize(data_list_.size(), data_list_.size());

  // convert times into times since last reading in list (everything is done
  // relative to the last readings values to minimize linearizion and wrap
  // round issues when predicting into the future, it also improves the
  // validity of the zero mean approximation made to simplify the gp)
  time_diffs_.resize(data_list_.size());
  std::vector<double>::iterator time_it = time_diffs_.begin();
  for (const Data& data : data_list_) {
    *time_it = (data.first - data_list_.back().first).toSec();
    ++time_it;
  }

  // loop over computing elements of the covariance using the specified
  // kernel. Only lower triangular elements are computed as the others won't
  // be used.
  for (size_t i = 0; i < data_list_.size(); ++i) {
    for (size_t j = 0; j <= i; ++j) {
      K(i, j) = kernel(time_diffs_[i], time_diffs_[j], i == j);
    }
  }

  // calculate mu (K-1*(data - mean))
  Eigen::VectorXd data_values;
  data_values.resize(data_list_.size());

  // covert data into change between this reading and the last one
  std::list<Data>::iterator data_it = data_list_.begin();
  for (size_t i = 0; i < data_list_.size(); ++i) {
    if (is_angle_) {
      data_values(i) = std::min(data_it->second - data_list_.back().second,
                                data_it->second + data_list_.back().second);
    } else {
      data_values(i) = data_it->second - data_list_.back().second;
    }

    ++data_it;
  }

  mu_ = K.llt().solve(data_values);

  ready_to_predict_ = true;
}

// get the prediction at the specified time
double GaussianProcess::predict(const ros::Time& prediction_time) {
  if (!ready_to_predict_) {
    throw std::runtime_error("Prediction requested before system was ready");
  }

  double prediction_time_diff =
      (prediction_time - data_list_.back().first).toSec();

  if (prediction_time_diff > time_diff_warning_) {
    ROS_WARN_STREAM(
        "Predicting "
        << prediction_time_diff
        << " seconds after last reading, result may be highly inaccurate");
  }
  if ((prediction_time - data_list_.front().first).toSec() <
      -time_diff_warning_) {
    ROS_WARN_STREAM(
        "Predicting "
        << prediction_time_diff
        << " seconds before first reading, result may be highly inaccurate");
  }

  double predicted_value = data_list_.back().second;
  for (size_t i = 0; i < data_list_.size(); ++i) {
    predicted_value += kernel(time_diffs_[i], prediction_time_diff) * mu_(i);
  }
  return predicted_value;
}