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
  transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
      "output_transform", queue_size);
}

void GP3::transformCallback(
    const geometry_msgs::TransformStampedConstPtr& transform_msg) {

  static int i = 0;
  if(i < 3){
    ++i;
    return;
  }
  i = 0;
  transform_predictor_->addTransformMeasurement(*transform_msg);
  frame_id_ = transform_msg->header.frame_id;
  child_frame_id_ = transform_msg->child_frame_id + "_gp3";
  have_data_ = true;
}

void GP3::timerCallback(const ros::TimerEvent&) {

  if(!have_data_){
    return;
  }

  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header.seq = seq_++;
  transform_msg.header.frame_id = frame_id_;
  transform_msg.child_frame_id = child_frame_id_;
  transform_msg.header.stamp =
      ros::Time::now() + ros::Duration(prediction_time_offset_);

  kindr::minimal::QuatTransformation transform;
  transform_predictor_->predictTransform(transform_msg.header.stamp,
                                         &transform);

  transform = transform * offset_transform_;
  tf::transformKindrToMsg(transform, &(transform_msg.transform));

  transform_pub_.publish(transform_msg);

  if (publish_tf_) {
    br_.sendTransform(transform_msg);
  }
}