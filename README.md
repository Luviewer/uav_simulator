# uav_gazebo

**这里推荐使用Ubuntu20.04** 

#### 安装教程(重要)
1. 安装 `ardupilot` 链接：https://github.com/Luviewer/ardupilot
2. 安装`VSCODE`
3. `VSCODE`安装`taskrunner`插件

### 仿真教程
1. 运行ardupilot

```
cd ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```
2. 运行gazebo

# tsduav_c1
```roslaunch uav_gazebo spawn_tsduav_c1.launch```


1