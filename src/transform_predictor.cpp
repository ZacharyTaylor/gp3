#include "gp3/transform_predictor.h"
#include "minkindr_conversions/kindr_msg.h"

TransformPredictor::Config TransformPredictor::readConfig(
    ros::NodeHandle nh, bool group_lengthscale, bool group_signal_noise_sd) {
  Config config;

  if (group_lengthscale) {
    double position_lengthscale = readValue(nh, "position_lengthscale");
    double orientation_lengthscale = readValue(nh, "orientation_lengthscale");
    double linear_velocity_lengthscale =
        readValue(nh, "linear_velocity_lengthscale");
    double angular_velocity_lengthscale =
        readValue(nh, "angular_velocity_lengthscale");

    config.x_lengthscale = position_lengthscale;
    config.y_lengthscale = position_lengthscale;
    config.z_lengthscale = position_lengthscale;

    config.rx_lengthscale = orientation_lengthscale;
    config.ry_lengthscale = orientation_lengthscale;
    config.rz_lengthscale = orientation_lengthscale;

    config.vx_lengthscale = linear_velocity_lengthscale;
    config.vy_lengthscale = linear_velocity_lengthscale;
    config.vz_lengthscale = linear_velocity_lengthscale;

    config.wx_lengthscale = angular_velocity_lengthscale;
    config.wy_lengthscale = angular_velocity_lengthscale;
    config.wz_lengthscale = angular_velocity_lengthscale;

  } else {
    config.x_lengthscale = readValue(nh, "x_lengthscale");
    config.y_lengthscale = readValue(nh, "y_lengthscale");
    config.z_lengthscale = readValue(nh, "z_lengthscale");

    config.rx_lengthscale = readValue(nh, "rx_lengthscale");
    config.ry_lengthscale = readValue(nh, "ry_lengthscale");
    config.rz_lengthscale = readValue(nh, "rz_lengthscale");

    config.vx_lengthscale = readValue(nh, "vx_lengthscale");
    config.vy_lengthscale = readValue(nh, "vy_lengthscale");
    config.vz_lengthscale = readValue(nh, "vz_lengthscale");

    config.wx_lengthscale = readValue(nh, "wx_lengthscale");
    config.wy_lengthscale = readValue(nh, "wy_lengthscale");
    config.wz_lengthscale = readValue(nh, "wz_lengthscale");
  }

  if (group_signal_noise_sd) {
    double position_signal_noise_sd = readValue(nh, "position_signal_noise_sd");
    double orientation_signal_noise_sd =
        readValue(nh, "orientation_signal_noise_sd");
    double linear_velocity_signal_noise_sd =
        readValue(nh, "linear_velocity_signal_noise_sd");
    double angular_velocity_signal_noise_sd =
        readValue(nh, "angular_velocity_signal_noise_sd");

    config.x_signal_noise_sd = position_signal_noise_sd;
    config.y_signal_noise_sd = position_signal_noise_sd;
    config.z_signal_noise_sd = position_signal_noise_sd;

    config.rx_signal_noise_sd = orientation_signal_noise_sd;
    config.ry_signal_noise_sd = orientation_signal_noise_sd;
    config.rz_signal_noise_sd = orientation_signal_noise_sd;

    config.vx_signal_noise_sd = linear_velocity_signal_noise_sd;
    config.vy_signal_noise_sd = linear_velocity_signal_noise_sd;
    config.vz_signal_noise_sd = linear_velocity_signal_noise_sd;

    config.wx_signal_noise_sd = angular_velocity_signal_noise_sd;
    config.wy_signal_noise_sd = angular_velocity_signal_noise_sd;
    config.wz_signal_noise_sd = angular_velocity_signal_noise_sd;

  } else {
    config.x_signal_noise_sd = readValue(nh, "x_signal_noise_sd");
    config.y_signal_noise_sd = readValue(nh, "y_signal_noise_sd");
    config.z_signal_noise_sd = readValue(nh, "z_signal_noise_sd");

    config.rx_signal_noise_sd = readValue(nh, "rx_signal_noise_sd");
    config.ry_signal_noise_sd = readValue(nh, "ry_signal_noise_sd");
    config.rz_signal_noise_sd = readValue(nh, "rz_signal_noise_sd");

    config.vx_signal_noise_sd = readValue(nh, "vx_signal_noise_sd");
    config.vy_signal_noise_sd = readValue(nh, "vy_signal_noise_sd");
    config.vz_signal_noise_sd = readValue(nh, "vz_signal_noise_sd");

    config.wx_signal_noise_sd = readValue(nh, "wx_signal_noise_sd");
    config.wy_signal_noise_sd = readValue(nh, "wy_signal_noise_sd");
    config.wz_signal_noise_sd = readValue(nh, "wz_signal_noise_sd");
  }

  nh.param("max_data_points", config.max_data_points, kDefaultMaxDataPoints);
  if (config.max_data_points <= 1) {
    throw std::runtime_error("Max data points value must be greater than 1");
  }
  nh.param("max_data_difference", config.max_data_difference,
           kDefaultMaxDataDifference);
  if (config.max_data_difference < 0) {
    throw std::runtime_error(
        "Max data difference must be greater than or equal to 0");
  }
  nh.param("time_diff_warning", config.time_diff_warning,
           kDefaultTimeDiffWarning);

  return config;
}

double TransformPredictor::readValue(ros::NodeHandle nh,
                                     std::string value_name) {
  double value;
  if (!nh.getParam(value_name, value)) {
    throw std::runtime_error(std::string("Could not find ") + value_name +
                             " in parameter server");
  }
  if (value <= 0) {
    throw std::runtime_error(std::string("Value for ") + value_name +
                             " must be greater than 0");
  }
  return value;
}

TransformPredictor::TransformPredictor(const Config& config)
    : time_diff_warning_(config.time_diff_warning),
      max_data_difference_(config.max_data_difference),
      max_data_points_(config.max_data_points),
      x_gp_(config.x_lengthscale, config.x_signal_noise_sd),
      y_gp_(config.y_lengthscale, config.y_signal_noise_sd),
      z_gp_(config.z_lengthscale, config.z_signal_noise_sd),
      rx_gp_(config.rx_lengthscale, config.rx_signal_noise_sd),
      ry_gp_(config.ry_lengthscale, config.ry_signal_noise_sd),
      rz_gp_(config.rz_lengthscale, config.rz_signal_noise_sd),
      vx_gp_(config.vx_lengthscale, config.vx_signal_noise_sd),
      vy_gp_(config.vy_lengthscale, config.vy_signal_noise_sd),
      vz_gp_(config.vz_lengthscale, config.vz_signal_noise_sd),
      wx_gp_(config.wx_lengthscale, config.wx_signal_noise_sd),
      wy_gp_(config.wy_lengthscale, config.wy_signal_noise_sd),
      wz_gp_(config.wz_lengthscale, config.wz_signal_noise_sd) {}

void TransformPredictor::addTransformMeasurement(
    const geometry_msgs::TransformStamped& transform_msg) {
  // convert to current transform
  TransformData current_transform_data;
  current_transform_data.timestamp = transform_msg.header.stamp;
  tf::transformMsgToKindr(transform_msg.transform,
                          &(current_transform_data.transform));

  // add to data points list
  transform_data_list_.push_back(current_transform_data);
  while (transform_data_list_.size() > max_data_points_) {
    transform_data_list_.pop_front();
  }

  if(transform_data_list_.size() <= 1){
    return;
  }

  // setup containers
  std::vector<Eigen::VectorXd> relative_log_transforms;
  Eigen::VectorXd relative_times;

  relative_log_transforms.resize(6);
  for (Eigen::VectorXd& relative_log_transform : relative_log_transforms) {
    relative_log_transform.resize(transform_data_list_.size());
  }
  relative_times.resize(transform_data_list_.size());

  // get all transforms relative to last one (helps with angle wrap round and
  // makes Gaussian 0 mean assumption more valid)
  std::list<TransformData>::const_iterator transform_it =
      transform_data_list_.begin();
  for (size_t i = 0; i < transform_data_list_.size(); ++i) {
    Eigen::Matrix<double, 6, 1> log_transform =
        (transform_data_list_.back().transform.inverse() *
         transform_it->transform)
            .log();
    for (size_t j = 0; j < relative_log_transforms.size(); ++j) {
      relative_log_transforms[j][i] = log_transform[j];
    }
    relative_times[i] =
        (transform_it->timestamp - transform_data_list_.back().timestamp)
            .toSec();

    ++transform_it;
  }

  // now do the same for body velocities
  std::vector<Eigen::VectorXd> velocities;
  Eigen::VectorXd velocity_times;

  velocities.resize(6);
  for (Eigen::VectorXd& velocity : velocities) {
    velocity.resize(transform_data_list_.size() - 1);
  }
  velocity_times.resize(transform_data_list_.size() - 1);

  std::list<TransformData>::const_iterator current_transform_it =
      transform_data_list_.begin();
  ++current_transform_it;
  std::list<TransformData>::const_iterator previous_transform_it =
      transform_data_list_.begin();
  for (size_t i = 0; i < transform_data_list_.size() - 1; ++i) {
    Eigen::Matrix<double, 6, 1> velocity =
        (previous_transform_it->transform.inverse() *
         current_transform_it->transform)
            .log() /
        (current_transform_it->timestamp - previous_transform_it->timestamp)
            .toSec();

    for (size_t j = 0; j < velocities.size(); ++j) {
      velocities[j][i] = velocity[j];
    }
    velocity_times[i] = ((current_transform_it->timestamp -
                          transform_data_list_.back().timestamp)
                             .toSec() +
                         (previous_transform_it->timestamp -
                          transform_data_list_.back().timestamp)
                             .toSec()) /
                        2;

    ++previous_transform_it;
    ++current_transform_it;
  }

  bool crop = false;
  size_t good_measurements = 1;

  // go through and crop data to get rid of any jumps (such as an angle
  // wrapping)
  for (int i = transform_data_list_.size() - 2; i >= 0; --i) {
    for (const Eigen::VectorXd& relative_log_transform :
         relative_log_transforms) {
      double difference =
          std::abs(relative_log_transform[i + 1] - relative_log_transform[i]);

      if (difference > max_data_difference_) {
        crop = true;
        break;
      }
    }
    if (crop) {
      break;
    }
    ++good_measurements;
  }

  // crop unreliable data
  if (crop) {
    for (Eigen::VectorXd& relative_log_transform : relative_log_transforms) {
      Eigen::VectorXd temp = relative_log_transform.tail(good_measurements);
      relative_log_transform = temp;
    }

    Eigen::VectorXd temp = relative_times.tail(good_measurements);
    relative_times = temp;
  }

  x_gp_.computeCovariance(relative_times, relative_log_transforms[0]);
  y_gp_.computeCovariance(relative_times, relative_log_transforms[1]);
  z_gp_.computeCovariance(relative_times, relative_log_transforms[2]);
  rx_gp_.computeCovariance(relative_times, relative_log_transforms[3]);
  ry_gp_.computeCovariance(relative_times, relative_log_transforms[4]);
  rz_gp_.computeCovariance(relative_times, relative_log_transforms[5]);

  vx_gp_.computeCovariance(velocity_times, velocities[0]);
  vy_gp_.computeCovariance(velocity_times, velocities[1]);
  vz_gp_.computeCovariance(velocity_times, velocities[2]);
  wx_gp_.computeCovariance(velocity_times, velocities[3]);
  wy_gp_.computeCovariance(velocity_times, velocities[4]);
  wz_gp_.computeCovariance(velocity_times, velocities[5]);
}

void TransformPredictor::predict(
    const ros::Time& prediction_time,
    Eigen::Matrix<double, 6, 1>* relative_transform_log,
    Eigen::Matrix<double, 6, 1>* relative_velocity) {
  double relative_prediction_time =
      (prediction_time - transform_data_list_.back().timestamp).toSec();

  if (relative_prediction_time > time_diff_warning_) {
    ROS_WARN_STREAM(
        "Predicting "
        << relative_prediction_time
        << " seconds after last reading, result may be highly inaccurate");
  }
  if ((prediction_time - transform_data_list_.front().timestamp).toSec() <
      -time_diff_warning_) {
    ROS_WARN_STREAM(
        "Predicting "
        << -relative_prediction_time
        << " seconds before first reading, result may be highly inaccurate");
  }

  relative_transform_log->coeffRef(0) = x_gp_.predict(relative_prediction_time);
  relative_transform_log->coeffRef(1) = y_gp_.predict(relative_prediction_time);
  relative_transform_log->coeffRef(2) = z_gp_.predict(relative_prediction_time);
  relative_transform_log->coeffRef(3) =
      rx_gp_.predict(relative_prediction_time);
  relative_transform_log->coeffRef(4) =
      ry_gp_.predict(relative_prediction_time);
  relative_transform_log->coeffRef(5) =
      rz_gp_.predict(relative_prediction_time);

  relative_velocity->coeffRef(0) = vx_gp_.predict(relative_prediction_time);
  relative_velocity->coeffRef(1) = vy_gp_.predict(relative_prediction_time);
  relative_velocity->coeffRef(2) = vz_gp_.predict(relative_prediction_time);
  relative_velocity->coeffRef(3) = wx_gp_.predict(relative_prediction_time);
  relative_velocity->coeffRef(4) = wy_gp_.predict(relative_prediction_time);
  relative_velocity->coeffRef(5) = wz_gp_.predict(relative_prediction_time);
}

void TransformPredictor::predict(const ros::Time& prediction_time,
                                 kindr::minimal::QuatTransformation* transform,
                                 Eigen::Matrix<double, 6, 1>* velocity) {
  Eigen::Matrix<double, 6, 1> relative_transform_log;
  predict(prediction_time, &relative_transform_log, velocity);
  *transform = kindr::minimal::QuatTransformation::exp(relative_transform_log);
  *transform = transform_data_list_.back().transform * (*transform);
}