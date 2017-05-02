#include "gp3/transform_predictor.h"
#include "minkindr_conversions/kindr_msg.h"

TransformPredictor::Config TransformPredictor::readConfig(
    ros::NodeHandle nh, bool group_lengthscale, bool group_signal_noise_sd) {
  Config config;

  if (group_lengthscale) {
    double position_lengthscale = readValue(nh, "position_lengthscale");
    double orientation_lengthscale = readValue(nh, "orientation_lengthscale");

    config.x_lengthscale_ = position_lengthscale;
    config.y_lengthscale_ = position_lengthscale;
    config.z_lengthscale_ = position_lengthscale;

    config.rx_lengthscale_ = orientation_lengthscale;
    config.ry_lengthscale_ = orientation_lengthscale;
    config.rz_lengthscale_ = orientation_lengthscale;
  } else {
    config.x_lengthscale_ = readValue(nh, "x_lengthscale");
    config.y_lengthscale_ = readValue(nh, "y_lengthscale");
    config.z_lengthscale_ = readValue(nh, "z_lengthscale");

    config.rx_lengthscale_ = readValue(nh, "rx_lengthscale");
    config.ry_lengthscale_ = readValue(nh, "ry_lengthscale");
    config.rz_lengthscale_ = readValue(nh, "rz_lengthscale");
  }

  if (group_signal_noise_sd) {
    double position_signal_noise_sd = readValue(nh, "position_signal_noise_sd");
    double orientation_signal_noise_sd =
        readValue(nh, "orientation_signal_noise_sd");

    config.x_signal_noise_sd_ = position_signal_noise_sd;
    config.y_signal_noise_sd_ = position_signal_noise_sd;
    config.z_signal_noise_sd_ = position_signal_noise_sd;

    config.rx_signal_noise_sd_ = orientation_signal_noise_sd;
    config.ry_signal_noise_sd_ = orientation_signal_noise_sd;
    config.rz_signal_noise_sd_ = orientation_signal_noise_sd;
  } else {
    config.x_signal_noise_sd_ = readValue(nh, "x_signal_noise_sd");
    config.y_signal_noise_sd_ = readValue(nh, "y_signal_noise_sd");
    config.z_signal_noise_sd_ = readValue(nh, "z_signal_noise_sd");

    config.rx_signal_noise_sd_ = readValue(nh, "rx_signal_noise_sd");
    config.ry_signal_noise_sd_ = readValue(nh, "ry_signal_noise_sd");
    config.rz_signal_noise_sd_ = readValue(nh, "rz_signal_noise_sd");
  }

  nh.param("max_data_points", config.max_data_points_, kDefaultMaxDataPoints);
  if (config.max_data_points_ <= 0) {
    throw std::runtime_error("Max data points value must be greater than 0");
  }
  nh.param("time_diff_warning", config.time_diff_warning_,
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
    : x_gp_(config.x_lengthscale_, config.x_signal_noise_sd_,
            config.max_data_points_, false, config.time_diff_warning_),
      y_gp_(config.y_lengthscale_, config.y_signal_noise_sd_,
            config.max_data_points_, false),
      z_gp_(config.z_lengthscale_, config.z_signal_noise_sd_,
            config.max_data_points_, false),
      rx_gp_(config.rx_lengthscale_, config.rx_signal_noise_sd_,
             config.max_data_points_, true),
      ry_gp_(config.ry_lengthscale_, config.ry_signal_noise_sd_,
             config.max_data_points_, true),
      rz_gp_(config.rz_lengthscale_, config.rz_signal_noise_sd_,
             config.max_data_points_, true) {}

void TransformPredictor::addTransformMeasurement(
    const geometry_msgs::TransformStamped& transform_msg) {
  kindr::minimal::QuatTransformation transform;
  tf::transformMsgToKindr(transform_msg.transform, &transform);

  Eigen::Matrix<double, 6, 1> transform_log = transform.log();
  x_gp_.addDataPoint(transform_msg.header.stamp, transform_log[0]);
  y_gp_.addDataPoint(transform_msg.header.stamp, transform_log[1]);
  z_gp_.addDataPoint(transform_msg.header.stamp, transform_log[2]);

  rx_gp_.addDataPoint(transform_msg.header.stamp, transform_log[3]);
  ry_gp_.addDataPoint(transform_msg.header.stamp, transform_log[4]);
  rz_gp_.addDataPoint(transform_msg.header.stamp, transform_log[5]);
}

void TransformPredictor::predictTransform(
    const ros::Time& prediction_time,
    Eigen::Matrix<double, 6, 1>* transform_log) {
  transform_log->coeffRef(0) = rx_gp_.predict(prediction_time);
  transform_log->coeffRef(1) = ry_gp_.predict(prediction_time);
  transform_log->coeffRef(2) = rz_gp_.predict(prediction_time);

  transform_log->coeffRef(3) = rx_gp_.predict(prediction_time);
  transform_log->coeffRef(4) = ry_gp_.predict(prediction_time);
  transform_log->coeffRef(5) = rz_gp_.predict(prediction_time);
}

void TransformPredictor::predictTransform(
    const ros::Time& prediction_time,
    kindr::minimal::QuatTransformation* transform) {
  Eigen::Matrix<double, 6, 1> transform_log;
  predictTransform(prediction_time, &transform_log);
  *transform = kindr::minimal::QuatTransformation::exp(transform_log);
}