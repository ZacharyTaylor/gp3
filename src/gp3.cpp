#include "gp3/gp3.h"

GP3::GP3(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      imu_predictor_(ImuPredictor::readConfig(nh_private_)) {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kDefaultQueueSize);

  nh_private_.param("prediction_time_offset", prediction_time_offset_,
                    kDefaultPredictionTimeOffset);

  imu_sub_ =
      nh_.subscribe("input_imu", queue_size, &GP3::imuCallback, this);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("output_imu", queue_size);
}

void GP3::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  imu_predictor_.addMeasurement(*msg);

  imu_pub_.publish(imu_predictor_.predict(
      ros::Time::now() + ros::Duration(prediction_time_offset_)));
}