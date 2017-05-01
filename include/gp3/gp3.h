#include <ros/ros.h>

#include "geometry_msgs/TransformStamped.h"
#include "gp3/gaussian_process.h"
#include "kindr/minimal/quat-transformation.h"
#include "minkindr_conversions/kindr_msg.h"

constexpr int kDefaultMaxDataPoints = 100;
constexpr double kDefaultTimeDiffWarning = 0.25;

class GP3 {
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
                           bool group_signal_noise_sd = true) {
    Config config;

    if (group_lengthscale) {
      double position_lengthscale = readValue(nh, "position_lengthscale");
      double orientation_lengthscale = readValue(nh, "orientation_lengthscale");

      config.x_lengthscale_ = position_lengthscale;
      config.y_lengthscale_ = position_lengthscale;
      config.z_lengthscale_ = position_lengthscale;

      config.rx_lengthscale_ = orientation_lengthscale;
      config.ry_lengthscale_ = orientation_lengthscale;
      config.rz_lengthscale_ = orientation_lengthscale;
    } else {
      config.x_lengthscale_ = readValue(nh, "x_lengthscale");
      config.y_lengthscale_ = readValue(nh, "y_lengthscale");
      config.z_lengthscale_ = readValue(nh, "z_lengthscale");

      config.rx_lengthscale_ = readValue(nh, "rx_lengthscale");
      config.ry_lengthscale_ = readValue(nh, "ry_lengthscale");
      config.rz_lengthscale_ = readValue(nh, "rz_lengthscale");
    }

    if (group_signal_noise_sd) {
      double position_signal_noise_sd =
          readValue(nh, "position_signal_noise_sd");
      double orientation_signal_noise_sd =
          readValue(nh, "orientation_signal_noise_sd");

      config.x_signal_noise_sd_ = position_signal_noise_sd;
      config.y_signal_noise_sd_ = position_signal_noise_sd;
      config.z_signal_noise_sd_ = position_signal_noise_sd;

      config.rx_signal_noise_sd_ = orientation_signal_noise_sd;
      config.ry_signal_noise_sd_ = orientation_signal_noise_sd;
      config.rz_signal_noise_sd_ = orientation_signal_noise_sd;
    } else {
      config.x_signal_noise_sd_ = readValue(nh, "x_signal_noise_sd");
      config.y_signal_noise_sd_ = readValue(nh, "y_signal_noise_sd");
      config.z_signal_noise_sd_ = readValue(nh, "z_signal_noise_sd");

      config.rx_signal_noise_sd_ = readValue(nh, "rx_signal_noise_sd");
      config.ry_signal_noise_sd_ = readValue(nh, "ry_signal_noise_sd");
      config.rz_signal_noise_sd_ = readValue(nh, "rz_signal_noise_sd");
    }

    nh.param("max_data_points", config.max_data_points_, kDefaultMaxDataPoints);
    if (config.max_data_points_ <= 0) {
      throw std::runtime_error("Max data points value must be greater than 0");
    }
    nh.param("time_diff_warning", config.time_diff_warning_,
             kDefaultTimeDiffWarning);

    return config;
  }

  static double readValue(ros::NodeHandle nh, std::string value_name) {
    double value;
    if (nh.getParam(value_name, value)) {
      throw std::runtime_error(std::string("Could not find ") + value_name +
                               " in parameter server");
    }
    if (value <= 0) {
      throw std::runtime_error(std::string("Value for ") + value_name +
                               " must be greater than 0");
    }
    return value;
  }

  GP3(const Config& config)
      : x_gp_(SqExpKernel(config.x_lengthscale_, config.x_signal_noise_sd_),
              config.max_data_points_, config.time_diff_warning_),
        y_gp_(SqExpKernel(config.y_lengthscale_, config.y_signal_noise_sd_),
              config.max_data_points_),
        z_gp_(SqExpKernel(config.z_lengthscale_, config.z_signal_noise_sd_),
              config.max_data_points_),
        rx_gp_(SqExpAngleWrapKernel(config.rx_lengthscale_,
                                    config.rx_signal_noise_sd_),
               config.max_data_points_),
        ry_gp_(SqExpAngleWrapKernel(config.ry_lengthscale_,
                                    config.ry_signal_noise_sd_),
               config.max_data_points_),
        rz_gp_(SqExpAngleWrapKernel(config.rz_lengthscale_,
                                    config.rz_signal_noise_sd_),
               config.max_data_points_) {}

  void addTransformMeasurement(
      const geometry_msgs::TransformStamped& tform_msg) {
    kindr::minimal::QuatTransformation tform;
    tf::transformMsgToKindr(tform_msg.Transform, &tform);

    Eigen::Matrix<double, 6, 1> tform_log = tform.log();
    x_gp_.addDataPoint(tform.header.stamp, tform_log[0]);
    y_gp_.addDataPoint(tform.header.stamp, tform_log[1]);
    z_gp_.addDataPoint(tform.header.stamp, tform_log[2]);

    rx_gp_.addDataPoint(tform.header.stamp, tform_log[3]);
    ry_gp_.addDataPoint(tform.header.stamp, tform_log[4]);
    rz_gp_.addDataPoint(tform.header.stamp, tform_log[5]);
  }

  void predictTransform(const ros::Time& prediction_time,
                        Eigen::Matrix<double, 6, 1>* tform_log) {
    x_gp_.predict(prediction_time, &(tform_log->coeffRef(0)));
    y_gp_.predict(prediction_time, &(tform_log->coeffRef(1)));
    z_gp_.predict(prediction_time, &(tform_log->coeffRef(2)));

    rx_gp_.predict(prediction_time, &(tform_log->coeffRef(3)));
    ry_gp_.predict(prediction_time, &(tform_log->coeffRef(4)));
    rz_gp_.predict(prediction_time, &(tform_log->coeffRef(5)));
  }

  void predictTransform(const ros::Time& prediction_time,
                        kindr::minimal::QuatTransformation* tform) {
    Eigen::Matrix<double, 6, 1> tform_log;
    predictTransform(prediction_time, &tform_log);
    *tform = kindr::minimal::QuatTransformation::exp(tform_log);
  }

  void predictTransform(const ros::Time& prediction_time,
                        geometry_msgs::TransformStamped* tform_msg) {
    kindr::minimal::QuatTransformation tform;
    predictTransform(prediction_time, &tform);
    
    tf::transformKindrToMsg(tform, &(tform_msg->transform));
    tform_msg->header.stamp = prediction_time;
  }

 private:
  GaussianProcess x_gp_;
  GaussianProcess y_gp_;
  GaussianProcess z_gp_;

  GaussianProcess rx_gp_;
  GaussianProcess ry_gp_;
  GaussianProcess rz_gp_;
};