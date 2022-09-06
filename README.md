# uav_gazebo

#### 安装教程(重要)

```
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models/arduwoodpecker/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models/tsduav_quad/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models/tsduav_t4/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models/electric_tower/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc

source ~/.bashrc
```

### 仿真教程
```
cd ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```