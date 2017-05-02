#ifndef GP3_TRANSFORM_PREDICTOR_H
#define GP3_TRANSFORM_PREDICTOR_H

#include <ros/ros.h>

#include "geometry_msgs/TransformStamped.h"
#include "kindr/minimal/quat-transformation.h"

#include "gp3/gaussian_process.h"


constexpr int kDefaultMaxDataPoints = 10;
constexpr double kDefaultTimeDiffWarning = 0.25;

class TransformPredictor {
 public:
  struct Config {
    double x_lengthscale_;
    double y_lengthscale_;
    double z_lengthscale_;

    double rx_lengthscale_;
    double ry_lengthscale_;
    double rz_lengthscale_;

    double x_signal_noise_sd_;
    double y_signal_noise_sd_;
    double z_signal_noise_sd_;

    double rx_signal_noise_sd_;
    double ry_signal_noise_sd_;
    double rz_signal_noise_sd_;

    int max_data_points_;
    double time_diff_warning_;
  };

  static Config readConfig(ros::NodeHandle nh, bool group_lengthscale = true,
                           bool group_signal_noise_sd = true);

  static double readValue(ros::NodeHandle nh, std::string value_name);

  TransformPredictor(const Config& config);

  void addTransformMeasurement(
      const geometry_msgs::TransformStamped& transform_msg);

  void predictTransform(const ros::Time& prediction_time,
                        Eigen::Matrix<double, 6, 1>* transform_log);

  void predictTransform(const ros::Time& prediction_time,
                        kindr::minimal::QuatTransformation* transform);

 private:
  GaussianProcess x_gp_;
  GaussianProcess y_gp_;
  GaussianProcess z_gp_;

  GaussianProcess rx_gp_;
  GaussianProcess ry_gp_;
  GaussianProcess rz_gp_;
};

#endif //GP3_TRANSFORM_PREDICTOR_H