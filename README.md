# uav_gazebo

#### 安装教程


```
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/worlds:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc
source ~/.bashrc
```

```
cd ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```