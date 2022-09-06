# uav_gazebo

**这里推荐使用Ubuntu20.04** 

#### 安装教程(重要)

```
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models/arduwoodpecker/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models/tsduav_quad/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models/tsduav_t4/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models/electric_tower/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
   

如果增加其他模型，请按照 `uav_simulator/uav_gazebo/models/xxx` 的写法添加模型，并增加在环境变量中，添加环境变量方式：
echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/uav_simulator/uav_gazebo/models/xxx/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc

source ~/.bashrc
```

### 仿真教程
```
cd ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```
