#ifndef GP3_GP3_H
#define GP3_GP3_H

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "gp3/transform_predictor.h"

constexpr int kDefaultQueueSize = 100;
constexpr double kDefaultPredictionRateHz = 100;
constexpr double kDefaultPredictionTimeOffset = 0;
constexpr bool kDefaultPublishTF = true;

class GP3 {
 public:
  GP3(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  void transformCallback(
      const geometry_msgs::TransformStampedConstPtr& transform_msg);

  void timerCallback(const ros::TimerEvent&);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber transform_sub_;
  ros::Publisher odometry_pub_;

  tf::TransformBroadcaster br_;

  ros::Timer timer_;

  double prediction_time_offset_;

  size_t num_data_;
  std::shared_ptr<TransformPredictor> transform_predictor_;

  kindr::minimal::QuatTransformation offset_transform_;

  bool publish_tf_;
  uint32_t seq_;
  std::string frame_id_;
  std::string child_frame_id_;
};

#endif //GP3_GP3_H