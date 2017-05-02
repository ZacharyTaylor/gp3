#ifndef GP3_TRANSFORM_PREDICTOR_H
#define GP3_TRANSFORM_PREDICTOR_H

#include <ros/ros.h>

#include "geometry_msgs/TransformStamped.h"
#include "kindr/minimal/quat-transformation.h"

#include "gp3/gaussian_process.h"

constexpr int kDefaultMaxDataPoints = 10;
constexpr double kDefaultMaxDataDifference = 1.0;
constexpr double kDefaultTimeDiffWarning = 0.25;

class TransformPredictor {
 public:
  struct Config {
    double x_lengthscale;
    double y_lengthscale;
    double z_lengthscale;

    double rx_lengthscale;
    double ry_lengthscale;
    double rz_lengthscale;

    double vx_lengthscale;
    double vy_lengthscale;
    double vz_lengthscale;

    double wx_lengthscale;
    double wy_lengthscale;
    double wz_lengthscale;

    double x_signal_noise_sd;
    double y_signal_noise_sd;
    double z_signal_noise_sd;

    double rx_signal_noise_sd;
    double ry_signal_noise_sd;
    double rz_signal_noise_sd;

    double vx_signal_noise_sd;
    double vy_signal_noise_sd;
    double vz_signal_noise_sd;

    double wx_signal_noise_sd;
    double wy_signal_noise_sd;
    double wz_signal_noise_sd;

    int max_data_points;
    double max_data_difference;
    double time_diff_warning;
  };

  static Config readConfig(ros::NodeHandle nh, bool group_lengthscale = true,
                           bool group_signal_noise_sd = true);

  static double readValue(ros::NodeHandle nh, std::string value_name);

  TransformPredictor(const Config& config);

  void addTransformMeasurement(
      const geometry_msgs::TransformStamped& transform_msg);

  void predict(const ros::Time& prediction_time,
                        Eigen::Matrix<double, 6, 1>* transform_log, Eigen::Matrix<double, 6, 1>* velocity);

  void predict(const ros::Time& prediction_time,
                        kindr::minimal::QuatTransformation* transform, Eigen::Matrix<double, 6, 1>* velocity);

 private:
  struct TransformData {
    ros::Time timestamp;
    kindr::minimal::QuatTransformation transform;
  };

  const double time_diff_warning_;
  const double max_data_difference_;
  const int max_data_points_;
  
  std::list<TransformData> transform_data_list_;

  GaussianProcess x_gp_;
  GaussianProcess y_gp_;
  GaussianProcess z_gp_;

  GaussianProcess rx_gp_;
  GaussianProcess ry_gp_;
  GaussianProcess rz_gp_;

  GaussianProcess vx_gp_;
  GaussianProcess vy_gp_;
  GaussianProcess vz_gp_;

  GaussianProcess wx_gp_;
  GaussianProcess wy_gp_;
  GaussianProcess wz_gp_;
};

#endif  // GP3_TRANSFORM_PREDICTOR_H