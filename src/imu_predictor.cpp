#include "gp3/imu_predictor.h"

ImuPredictor::Config ImuPredictor::readConfig(const ros::NodeHandle& nh) {
  Config config;

  config.w_l = readValue(nh, "angular_velocity_lengthscale");
  config.a_l = readValue(nh, "linear_acceleration_lengthscale");
  config.w_sd = readValue(nh, "angular_velocity_noise_sd");
  config.a_sd = readValue(nh, "linear_acceleration_noise_sd");

  nh.param("max_data_points", config.max_data_points, kDefaultMaxDataPoints);
  if (config.max_data_points <= 0) {
    throw std::runtime_error("Max data points value must be greater than 0");
  }

  return config;
}

double ImuPredictor::readValue(const ros::NodeHandle& nh,
                               const std::string& value_name) {
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

ImuPredictor::ImuPredictor(const Config& config)
    : wx_gp_(config.max_data_points, config.w_l, config.w_sd),
      wy_gp_(config.max_data_points, config.w_l, config.w_sd),
      wz_gp_(config.max_data_points, config.w_l, config.w_sd),
      ax_gp_(config.max_data_points, config.a_l, config.a_sd),
      ay_gp_(config.max_data_points, config.a_l, config.a_sd),
      az_gp_(config.max_data_points, config.a_l, config.a_sd) {}

void ImuPredictor::addMeasurement(const sensor_msgs::Imu& msg) {
  frame_id_ = msg.header.frame_id;

  wx_gp_.addDataPoint(msg.header.stamp, msg.angular_velocity.x);
  wy_gp_.addDataPoint(msg.header.stamp, msg.angular_velocity.y);
  wz_gp_.addDataPoint(msg.header.stamp, msg.angular_velocity.z);

  ax_gp_.addDataPoint(msg.header.stamp, msg.linear_acceleration.x);
  ay_gp_.addDataPoint(msg.header.stamp, msg.linear_acceleration.y);
  az_gp_.addDataPoint(msg.header.stamp, msg.linear_acceleration.z);

  is_setup_ = false;
}

// note this is the most computationally expensive function and so is separated
// from prediction so that calling predict(ros::Time::now()) is as accurate
// and undelayed as possible
void ImuPredictor::setupPrediction() {
  if (is_setup_) {
    return;
  }

  wx_gp_.computeCovariance();
  wy_gp_.computeCovariance();
  wz_gp_.computeCovariance();

  ax_gp_.computeCovariance();
  ay_gp_.computeCovariance();
  az_gp_.computeCovariance();

  is_setup_ = true;
}

sensor_msgs::Imu ImuPredictor::predict(const ros::Time& prediction_time) {
  if (!is_setup_) {
    throw std::runtime_error(
        "Prediction requested before system was setup, run setupPrediction "
        "first");
  }

  sensor_msgs::Imu msg;
  msg.header.seq = seq_++;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = prediction_time;

  msg.angular_velocity.x = wx_gp_.predict(prediction_time);
  msg.angular_velocity.y = wy_gp_.predict(prediction_time);
  msg.angular_velocity.z = wz_gp_.predict(prediction_time);

  msg.linear_acceleration.x = ax_gp_.predict(prediction_time);
  msg.linear_acceleration.y = ay_gp_.predict(prediction_time);
  msg.linear_acceleration.z = az_gp_.predict(prediction_time);

  return msg;
}