# Elastic Tracker + Kinodynamic A* 启动与编译速查

## 1. 工作空间说明

Kinodynamic A* 修改版工作空间：

```bash
~/elastic_tracker_kino_ws
```

原版 Elastic Tracker 基线工作空间：

```bash
~/elastic_tracker_ws
```

当前主要开发分支：

```text
feature/kino-astar
```

---

# 2. 新终端加载 ROS 1 Noetic

每次新打开一个终端，如果出现：

```text
catkin_make: command not found
```

或者：

```text
roslaunch: command not found
```

先执行：

```bash
source /opt/ros/noetic/setup.bash
```

检查 ROS 是否加载成功：

```bash
echo $ROS_DISTRO
which roslaunch
which catkin_make
```

正常情况下应该看到：

```text
noetic
/opt/ros/noetic/bin/roslaunch
/opt/ros/noetic/bin/catkin_make
```

> 如果提示 `catkin_make: command not found`，不要重新安装 catkin，先检查是否执行了：
>
> ```bash
> source /opt/ros/noetic/setup.bash
> ```

---

# 3. 编译 Elastic Tracker + Kinodynamic A*

## 3.1 进入工作空间

```bash
cd ~/elastic_tracker_kino_ws
```

## 3.2 加载 ROS Noetic

```bash
source /opt/ros/noetic/setup.bash
```

## 3.3 加载 CUDA 12.8

```bash
export PATH=/usr/local/cuda-12.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH
```

可以检查 CUDA：

```bash
nvcc --version
```

## 3.4 编译工程

```bash
catkin_make \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.8
```

编译成功后通常可以看到：

```text
[100%] Built target planning_nodelet
```

以及：

```text
Built target kino_astar
```

## 3.5 加载当前工作空间

编译完成后执行：

```bash
source ~/elastic_tracker_kino_ws/devel/setup.bash
```

---

# 4. 一套完整的编译命令

新开终端后，可以直接依次执行：

```bash
source /opt/ros/noetic/setup.bash

cd ~/elastic_tracker_kino_ws

export PATH=/usr/local/cuda-12.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH

catkin_make \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.8

source devel/setup.bash
```

---

# 5. Elastic Tracker 四终端启动流程

Elastic Tracker 仿真需要打开 **4 个终端**。

启动顺序建议为：

```text
终端 1：RViz + 地图
        ↓
终端 2：Target
        ↓
终端 3：Drone0 + Planning
        ↓
终端 4：Trigger
        ↓
RViz 点击 2D Nav Goal
```

---

# 6. 终端 1：启动 RViz 与地图环境

打开第一个终端：

```bash
cd ~/elastic_tracker_kino_ws

source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch mapping rviz_sim.launch
```

作用：

- 启动 RViz
- 加载仿真环境
- 加载全局地图 / 点云
- 提供无人机传感器仿真所需要的环境

---

# 7. 终端 2：启动移动目标 Target

打开第二个终端：

```bash
cd ~/elastic_tracker_kino_ws

source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch planning fake_target.launch
```

作用：

- 启动被无人机追踪的移动目标 `target`

在 RViz 中点击：

```text
2D Nav Goal
```

设置的是：

```text
Target 的运动目标
```

不是：

```text
Drone0 的运动目标
```

---

# 8. 终端 3：启动 Drone0 与 Elastic Tracker

打开第三个终端：

```bash
cd ~/elastic_tracker_kino_ws

source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch planning simulation1.launch
```

这个终端最重要。

主要包含：

- Drone0
- Mapping
- Planning
- Trajectory Server
- Controller
- Target EKF
- Kinodynamic A*

Kinodynamic A* 的主要调试信息都在这个终端查看。

例如初始化成功：

```text
[KinodynamicAstar] map min:  ...
[KinodynamicAstar] map max:  ...
[KinodynamicAstar] map resolution: 0.15
[KinodynamicAstar] search resolution: 0.1
[KinodynamicAstar] initialization finished.
```

Kinodynamic A* 搜索时：

```text
========== Kinodynamic A* Test ==========

start position : ...
start velocity : ...
start accel    : ...
goal position  : ...

=========================================
```

搜索成功：

```text
[KinodynamicAstar Test] SUCCESS
```

---

# 9. 终端 4：发送 Trigger

打开第四个终端：

```bash
cd ~/elastic_tracker_kino_ws

source /opt/ros/noetic/setup.bash
source devel/setup.bash

./sh_utils/pub_triger.sh
```

如果出现：

```text
publishing and latching message. Press ctrl-C to terminate
```

属于正常现象。

保持该终端运行即可。

---

# 10. 完整启动顺序

## Step 1：终端 1

```bash
roslaunch mapping rviz_sim.launch
```

## Step 2：终端 2

```bash
roslaunch planning fake_target.launch
```

## Step 3：终端 3

```bash
roslaunch planning simulation1.launch
```

## Step 4：终端 4

```bash
./sh_utils/pub_triger.sh
```

## Step 5：RViz

使用：

```text
2D Nav Goal
```

给 Target 设置运动目标。

然后观察：

```text
终端 3
```

中的规划日志。

---

# 11. 常用 ROS 检查命令

## 查看 ROS 版本

```bash
echo $ROS_DISTRO
```

正常：

```text
noetic
```

---

## 查看 roslaunch 路径

```bash
which roslaunch
```

正常：

```text
/opt/ros/noetic/bin/roslaunch
```

---

## 查看 catkin_make 路径

```bash
which catkin_make
```

正常：

```text
/opt/ros/noetic/bin/catkin_make
```

---

## 查看 ROS Package 路径

```bash
echo $ROS_PACKAGE_PATH
```

---

## 查看所有 Topic

```bash
rostopic list
```

---

## 查看 Drone0 相关 Topic

```bash
rostopic list | grep drone0
```

---

## 检查深度图是否发布

```bash
rostopic hz /drone0/depth
```

---

## 检查膨胀地图是否发布

```bash
rostopic hz /drone0/gridmap_inflate
```

---

## 检查无人机里程计

```bash
rostopic hz /drone0/odom
```

---

## 查看无人机实时里程计

```bash
rostopic echo /drone0/odom
```

---

## 查看轨迹 Topic

```bash
rostopic hz /drone0/trajectory
```

---

## 查看 Target EKF

```bash
rostopic hz /target_ekf_odom
```

---

# 12. Git 常用命令

进入 Kinodynamic A* 修改版：

```bash
cd ~/elastic_tracker_kino_ws
```

查看当前分支：

```bash
git branch --show-current
```

当前应为：

```text
feature/kino-astar
```

查看当前修改：

```bash
git status
```

查看最近提交：

```bash
git log --oneline -5
```

暂存修改：

```bash
git add .
```

提交：

```bash
git commit -m "提交说明"
```

推送到 GitHub：

```bash
git push
```

---

# 13. 两个工作空间不要混淆

## 原版基线

```bash
~/elastic_tracker_ws
```

用途：

```text
保存成功复现的原版 Elastic Tracker
```

尽量不要继续修改。

---

## Kinodynamic A* 修改版

```bash
~/elastic_tracker_kino_ws
```

用途：

```text
Kinodynamic A* 移植
地图接口适配
算法替换
实验与调试
```

当前主要开发工作都在这里进行。

---

# 14. 当前算法结构

原 Elastic Tracker：

```text
Target State
     ↓
Target Prediction
     ↓
findVisiblePath()
     ↓
A* Path
     ↓
Visible Region
     ↓
SFC
     ↓
MINCO
     ↓
Trajectory
     ↓
Drone0
```

当前已经旁路加入：

```text
Current Drone State
       ↓
   [p, v, a]
       ↓
Kinodynamic A*
       ↓
  OccGridMap
       ↓
   kino_path
```

当前阶段：

```text
原 A*：
正常控制无人机

Kinodynamic A*：
旁路测试
```

还没有正式让：

```text
kino_path
```

替换：

```text
原 A* path
```

---

# 15. 当前 Kinodynamic A* 已完成内容

目前已经完成：

- [x] Fast-Planner Kinodynamic A* 代码移植
- [x] 在 Elastic Tracker 中成功编译
- [x] 删除 `EDTEnvironment` 依赖
- [x] 删除 `SDFMap` 依赖
- [x] 接入 `mapping::OccGridMap`
- [x] 接入 `planning_nodelet`
- [x] 加载 Kinodynamic A* 参数
- [x] 获取 Elastic Tracker 地图
- [x] Kinodynamic A* 初始化成功
- [x] 第一次调用 `search()`
- [x] 第一次搜索成功
- [x] 返回 `REACH_END`
- [x] `getKinoTraj()` 成功输出路径

---

# 16. 当前第一次 Kino A* 测试结果

第一次搜索：

```text
start position :
0 0 2.0
```

目标：

```text
-0.225 0.075 2.925
```

结果：

```text
status = 2
```

对应：

```text
REACH_END
```

轨迹采样点：

```text
13 points
```

但是出现：

```text
Shot in first search loop!
```

说明目标距离较近，Kinodynamic A* 在第一次搜索循环中就通过 Shot Trajectory 直接连接终点。

因此下一阶段需要使用更远的目标验证真正的：

```text
Motion Primitive Expansion
```

---

# 17. 下一阶段目标

下一阶段：

```text
RViz 同屏显示

Elastic Tracker A*
        VS
Kinodynamic A*
```

并且：

```text
选择更远的 way_pts
        ↓
Kinodynamic A*
        ↓
Motion Primitive Expansion
        ↓
Collision Check
        ↓
Open Set
        ↓
Node Expansion
        ↓
Shot Trajectory
        ↓
Goal
```

重点观察：

```text
goal distance
status
visited nodes
path points
```

用于判断 Kinodynamic A* 是否真正进行了运动原语搜索。