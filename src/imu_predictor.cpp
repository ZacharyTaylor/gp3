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

ImuPredictor::ImuPredictor(const Config& config) {
  gp_vector_.emplace_back(config.max_data_points, config.w_l, config.w_sd);
  gp_vector_.emplace_back(config.max_data_points, config.w_l, config.w_sd);
  gp_vector_.emplace_back(config.max_data_points, config.w_l, config.w_sd);

  gp_vector_.emplace_back(config.max_data_points, config.a_l, config.a_sd);
  gp_vector_.emplace_back(config.max_data_points, config.a_l, config.a_sd);
  gp_vector_.emplace_back(config.max_data_points, config.a_l, config.a_sd);
}

void ImuPredictor::addMeasurement(const sensor_msgs::Imu& msg) {
  frame_id_ = msg.header.frame_id;

  addMeasurement(Element::WX, msg.header.stamp, msg.angular_velocity.x);
  addMeasurement(Element::WY, msg.header.stamp, msg.angular_velocity.y);
  addMeasurement(Element::WZ, msg.header.stamp, msg.angular_velocity.z);

  addMeasurement(Element::AX, msg.header.stamp, msg.linear_acceleration.x);
  addMeasurement(Element::AY, msg.header.stamp, msg.linear_acceleration.y);
  addMeasurement(Element::AZ, msg.header.stamp, msg.linear_acceleration.z);
}

void ImuPredictor::addMeasurement(const Element& element, const ros::Time& time,
                                  const double& value) {
  get_gp(element).addDataPoint(time, value);
}

GaussianProcess& ImuPredictor::get_gp(const Element& element){
    return gp_vector_[static_cast<size_t>(element)];
}

void ImuPredictor::setFrameID(std::string frame_id) { frame_id_ = frame_id; }

sensor_msgs::Imu ImuPredictor::predict(const ros::Time& prediction_time) {
  sensor_msgs::Imu msg;
  msg.header.seq = seq_++;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = prediction_time;

  msg.angular_velocity.x = get_gp(Element::WX).predict(prediction_time);
  msg.angular_velocity.y = get_gp(Element::WY).predict(prediction_time);
  msg.angular_velocity.z = get_gp(Element::WZ).predict(prediction_time);

  msg.linear_acceleration.x = get_gp(Element::AX).predict(prediction_time);
  msg.linear_acceleration.y = get_gp(Element::AY).predict(prediction_time);
  msg.linear_acceleration.z = get_gp(Element::AZ).predict(prediction_time);

  return msg;
}