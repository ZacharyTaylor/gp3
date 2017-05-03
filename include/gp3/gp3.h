#ifndef GP3_GP3_H
#define GP3_GP3_H

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include "gp3/imu_predictor.h"

constexpr int kDefaultQueueSize = 100;
constexpr double kDefaultPredictionRateHz = 100;
constexpr double kDefaultPredictionTimeOffset = 0;

class GP3 {
 public:
  GP3(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  void imuCallback(
      const sensor_msgs::ImuConstPtr& msg);

  void timerCallback(const ros::TimerEvent&);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_;

  ros::Timer timer_;

  double prediction_time_offset_;

  bool have_data_;
  ImuPredictor imu_predictor_;
};

#endif //GP3_GP3_H