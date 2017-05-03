#include "gp3/gp3.h"

GP3::GP3(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      have_data_(false),
      imu_predictor_(ImuPredictor::readConfig(nh_private_)) {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kDefaultQueueSize);

  double prediction_rate_hz;
  nh_private_.param("prediction_rate_hz", prediction_rate_hz,
                    kDefaultPredictionRateHz);

  nh_private_.param("prediction_time_offset", prediction_time_offset_,
                    kDefaultPredictionTimeOffset);

  timer_ = nh.createTimer(ros::Duration(1.0 / prediction_rate_hz),
                          &GP3::timerCallback, this);

  imu_sub_ =
      nh_.subscribe("input_imu", queue_size, &GP3::imuCallback, this);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("output_imu", queue_size);
}

void GP3::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  imu_predictor_.addMeasurement(*msg);
  have_data_ = true;
}

void GP3::timerCallback(const ros::TimerEvent&) {
  if (!have_data_) {
    return;
  }

  imu_predictor_.setupPrediction();
  imu_pub_.publish(imu_predictor_.predict(
      ros::Time::now() + ros::Duration(prediction_time_offset_)));
}