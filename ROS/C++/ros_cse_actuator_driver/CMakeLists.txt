cmake_minimum_required(VERSION 3.0.2)
project(cse_actuator_driver)


add_compile_options(-std=c++11 -lwiringPi -li2c)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nodelet
)

# find_package(Boost)
find_package(Eigen3)
add_definitions(${EIGEN_DEFINITIONS})
find_library(wiringPi_LIB wiringPi)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cse_actuator_driver_nodelet 
 CATKIN_DEPENDS nodelet roscpp
#  DEPENDS wiringPi
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # ${wiringPi_INCLUDE_DIR}
)

add_library(cse_actuator_driver_nodelet src/cse_actuator_driver_nodelet.cpp 
src/cse_actuator_driver.cpp 
src/PCA9685.cpp) 
target_link_libraries(cse_actuator_driver_nodelet 
${catkin_LIBRARIES}
# ${wiringPi_LIB}
-lwiringPi 
)


install(TARGETS cse_actuator_driver_nodelet 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)