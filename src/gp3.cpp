#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_xml.h>

#include "gp3/gp3.h"

GP3::GP3(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), have_data_(false), seq_(0) {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kDefaultQueueSize);

  double prediction_rate_hz;
  nh_private_.param("prediction_rate_hz", prediction_rate_hz,
                    kDefaultPredictionRateHz);

  nh_private_.param("take_every_nth_transform", take_every_nth_transform_,
                    kDefaultTakeEveryNthTransform);

  nh_private_.param("prediction_time_offset", prediction_time_offset_,
                    kDefaultPredictionTimeOffset);

  nh_private_.param("publish_tf", publish_tf_, kDefaultPublishTF);

  XmlRpc::XmlRpcValue raw_offset_transform;
  if (nh_private_.getParam("offset_transform", raw_offset_transform)) {
    kindr::minimal::xmlRpcToKindr(raw_offset_transform, &offset_transform_);
  }

  transform_predictor_ = std::make_shared<TransformPredictor>(
      TransformPredictor::readConfig(nh_private_));

  timer_ = nh.createTimer(ros::Duration(1.0 / prediction_rate_hz),
                          &GP3::timerCallback, this);

  transform_sub_ = nh_.subscribe("input_transform", queue_size,
                                 &GP3::transformCallback, this);
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(
      "output_odometry", queue_size);
}

void GP3::transformCallback(
    const geometry_msgs::TransformStampedConstPtr& transform_msg) {

  static int counter = 0;
  ++counter;

  if(counter >= take_every_nth_transform_){
    counter = 0;
    transform_predictor_->addTransformMeasurement(*transform_msg);
    frame_id_ = transform_msg->header.frame_id;
    child_frame_id_ = transform_msg->child_frame_id + "_gp3";
    have_data_ = true;
  }
}

void GP3::timerCallback(const ros::TimerEvent&) {
  if (!have_data_) {
    return;
  }

  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.seq = seq_++;
  odometry_msg.header.frame_id = frame_id_;
  odometry_msg.child_frame_id = child_frame_id_;
  odometry_msg.header.stamp =
      ros::Time::now() + ros::Duration(prediction_time_offset_);

  kindr::minimal::QuatTransformation transform;
  Eigen::Matrix<double, 6, 1> velocity;
  transform_predictor_->predict(odometry_msg.header.stamp, &transform,
                                &velocity);

  transform = transform * offset_transform_;
  tf::poseKindrToMsg(transform, &(odometry_msg.pose.pose));

  odometry_msg.twist.twist.linear.x = velocity[0];
  odometry_msg.twist.twist.linear.y = velocity[1];
  odometry_msg.twist.twist.linear.z = velocity[2];
  odometry_msg.twist.twist.angular.x = velocity[3];
  odometry_msg.twist.twist.angular.y = velocity[4];
  odometry_msg.twist.twist.angular.z = velocity[5];

  odometry_pub_.publish(odometry_msg);

  if (publish_tf_) {
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header = odometry_msg.header;
    tf::transformKindrToMsg(transform, &(transform_msg.transform));

    br_.sendTransform(transform_msg);
  }
}