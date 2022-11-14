cmake_minimum_required(VERSION 2.8.3)

project(uav_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  mav_msgs
  nav_msgs
  gazebo_msgs
)

# add_service_files(
#   FILES
#   ApplyBodyWrench.srv
# )

find_package(Eigen3 REQUIRED)

find_package(roslaunch)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES  pid_control
  CATKIN_DEPENDS roscpp  sensor_msgs  geometry_msgs mav_msgs nav_msgs gazebo_msgs
  DEPENDS Eigen3
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_library(pid_control
  src/pid_control.cpp
)


target_link_libraries(pid_control ${catkin_LIBRARIES})
add_dependencies(pid_control ${catkin_EXPORTED_TARGETS})


# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

add_executable(uav_pid_control_node src/uav_pid_control_node.cpp)
add_dependencies(uav_pid_control_node  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(uav_pid_control_node pid_control ${catkin_LIBRARIES})

add_executable(iwp_claw_node src/iwp_claw_node.cpp)
add_dependencies(iwp_claw_node  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(iwp_claw_node  ${catkin_LIBRARIES})

add_executable(iwp_mavros_node src/iwp_mavros_node.cpp)
add_dependencies(iwp_mavros_node  ${PROJECT_NAME}iwp_mavros_node)
target_link_libraries(iwp_mavros_node  ${catkin_LIBRARIES})

add_executable(uwb_node src/uwb_node.cpp)
add_dependencies(uwb_node  ${PROJECT_NAME}uwb_node)
target_link_libraries(uwb_node  ${catkin_LIBRARIES})


install(TARGETS pid_control 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
