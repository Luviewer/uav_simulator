cmake_minimum_required(VERSION 2.8.3)

project(uav_gazebo_plugin)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  mav_msgs
)


find_package(gazebo REQUIRED)
find_package(Eigen REQUIRED)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_plugins
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  # LIBRARIES pid 
  CATKIN_DEPENDS roscpp geometry_msgs gazebo_plugins mav_msgs
  DEPENDS Eigen3
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
include_directories(${mav_msgs_INCLUDE_DIRS})

add_library(HelloModelPlugin SHARED src/HelloModelPlugin.cc)
target_link_libraries(HelloModelPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(HelloWorldPlugin SHARED src/HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(MotorsPlugin SHARED src/MotorsPlugin.cc)
target_link_libraries(MotorsPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(ArduRotorNormPlugin SHARED src/ArduRotorNormPlugin.cc)
target_link_libraries(ArduRotorNormPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(ArduRotorTiltPlugin SHARED src/ArduRotorTiltPlugin.cc)
target_link_libraries(ArduRotorTiltPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(ArduRotorOctBoxPlugin SHARED src/ArduRotorOctBoxPlugin.cc)
target_link_libraries(ArduRotorOctBoxPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(position_controller SHARED src/position_controller.cc)
target_link_libraries(position_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(ArduRotorTiltPIDPlugin SHARED src/ArduRotorTiltPIDPlugin.cc)
target_link_libraries(ArduRotorTiltPIDPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(ArduRotorQuadOctPlugin SHARED src/ArduRotorQuadOctPlugin.cc)
target_link_libraries(ArduRotorQuadOctPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(ArduRotorBalancebotPlugin SHARED src/ArduRotorBalancebotPlugin.cc)
target_link_libraries(ArduRotorBalancebotPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})




