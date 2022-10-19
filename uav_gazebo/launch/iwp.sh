#! /bin/bash

roslaunch uav_gazebo spawn_iwp.launch &
sleep 5
echo "gazebo starting success!"

roslaunch uav_gazebo apm.launch &
sleep 1

# rosrun uav_control iwp_claw_node &
# sleep 1

wait 
exit 0
