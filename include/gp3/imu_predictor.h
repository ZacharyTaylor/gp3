#ifndef GP3_IMU_PREDICTOR_H
#define GP3_IMU_PREDICTOR_H

#include <ros/ros.h>

#include "gp3/gaussian_process.h"
#include "sensor_msgs/Imu.h"

constexpr int kDefaultMaxDataPoints = 10;

class ImuPredictor {
 public:
  struct Config {
    double w_l;
    double w_sd;

    double a_l;
    double a_sd;

    int max_data_points;
  };

  static Config readConfig(const ros::NodeHandle& nh);

  static double readValue(const ros::NodeHandle& nh,
                          const std::string& value_name);

  ImuPredictor(const Config& config);

  void addMeasurement(const sensor_msgs::Imu& msg);

  void setupPrediction();

  sensor_msgs::Imu predict(const ros::Time& prediction_time);

 private:
  uint64_t seq_;
  std::string frame_id_;

  bool is_setup_;

  GaussianProcess wx_gp_;
  GaussianProcess wy_gp_;
  GaussianProcess wz_gp_;

  GaussianProcess ax_gp_;
  GaussianProcess ay_gp_;
  GaussianProcess az_gp_;
};

#endif  // GP3_Imu_PREDICTOR_H