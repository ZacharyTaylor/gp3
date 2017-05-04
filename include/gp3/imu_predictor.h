#ifndef GP3_IMU_PREDICTOR_H
#define GP3_IMU_PREDICTOR_H

#include <ros/ros.h>

#include "gp3/gaussian_process.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"

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

  enum class Element : size_t {WX, WY, WZ, AX, AY, AZ};

  static Config readConfig(const ros::NodeHandle& nh);

  static double readValue(const ros::NodeHandle& nh,
                          const std::string& value_name);

  ImuPredictor(const Config& config);

  void addMeasurement(const sensor_msgs::Imu& msg);

  void addMeasurement(const Element& element, const ros::Time& time,
                                  const double& value);

  void setFrameID(std::string frame_id);

  sensor_msgs::Imu predict(const ros::Time& prediction_time);

 private:

  GaussianProcess& get_gp(const Element& element);

  uint64_t seq_;
  std::string frame_id_;

  std::vector<GaussianProcess> gp_vector_;
};

#endif  // GP3_Imu_PREDICTOR_H