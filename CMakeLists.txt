cmake_minimum_required(VERSION 2.8.3)
project(gp3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/gaussian_process.cpp
  src/gp3.cpp
  src/imu_predictor.cpp
)

############
# BINARIES #
############

cs_add_executable(gp3_node
  src/gp3_node.cpp
)
target_link_libraries(gp3_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
