#include "gp3/gp3.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gp3_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  GP3 gp3(nh, nh_private);

  ros::spin();

  return 0;
}
