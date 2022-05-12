cmake_minimum_required(VERSION 2.8.3)
project(uav_gazebo)

add_definitions(-std=c++11)

#--------------------------#
# Generation of SDF models #
#--------------------------#

# We need Gazebo version >= 3.0.0 to generate iris.sdf file
# (gz sdf ... command needs to be available)
find_package(gazebo REQUIRED)

find_package(PythonInterp REQUIRED)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs roscpp sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    roscpp
    sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

foreach(dir launch models resource worlds)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
