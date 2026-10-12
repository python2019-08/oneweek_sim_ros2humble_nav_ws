<!-- 2026/6/26 03:19 -->
<!-- <<Nav2_机器⼈⾃主导航_上_仿真⼀周指南>> -->
<!-- Generated from Markdown -->

<<Nav2 机器⼈⾃主导航（上）：ROS2 Humble + Nav2 + SLAM + EKF 仿真⼀周指南>>

⾯向对象：有 Linux、Python 或 C++ 基础，但没有完整⾃主导航项⽬落地经验的软件⼯程师。
⽬标：在 Gazebo 仿真环境中，从零搭建⼀套基于 ROS2 + Nav2 + SLAM + EKF 的机器⼈⾃主导航系统，完成第⼀个“建图 → 保存地图 → 加载地图 → 设置初始位姿 → 发送导航⽬标”的闭环。

这份⽂档不依赖任何既有项⽬仓库，不依赖任何真实硬件，作为⼀份可开⼯的实施⼿册使⽤。

# 1. ⽬标与边界

## 1.1 第⼀周⽬标（仿真）

* 在 Gazebo 仿真环境中搭建⼀个带 轮式底盘、2D激光雷达、IMU 的机器⼈。
* 完成仿真传感器接⼊、TF 配置、EKF 融合、SLAM 建图、Nav2 定点导航的完整闭环。
* ⽣成⼀套经过仿真验证的系统⻣架和参数基线（ robot_localization  +  Nav2 ），为第⼆周真机迁移做准备。

## 1.2 第⼀周不做什么

* 不接触任何真实硬件（底盘、雷达、IMU）。
* 不做室外导航。
* 不做⾼速运⾏。
* 不做视觉、语义、多传感器融合。
* 不做⾃动充电、巡检业务。

先把仿真闭环跑通，把 Topic / Frame / Launch / Parameter 的结构和调试⽅法摸清楚。第⼆周上真机时
仍然要重新确认传感器⽅向、时间戳、噪声、协⽅差、速度上限和安全策略，不能把仿真参数当成真机最终参数。

## 1.3 仿真环境的价值

* 零硬件⻛险：参数调错了，机器⼈不会撞墙。
* 可复现：同样的场景可以反复跑，便于对⽐参数效果。
* 并⾏开发：真机硬件还没到位时，软件团队可以先把导航链路跑通。
* 参数基线：在仿真⾥验证  max_vel_x 、 inflation_radius 、 footprint  等参数的结构和影响⽅向。迁移到真机后，这些参数仍需结合真实动⼒学、轮滑、传感器噪声和场地重新验证。
  
# 2. 软件栈

## 2.1 操作系统和 ROS2

本指南的命令和 Gazebo 插件示例按以下组合编写：
* Ubuntu 22.04
* ROS2 Humble
* Gazebo Classic 11

不建议初学者在同⼀份⼿册⾥混⽤ Ubuntu 24.04 + ROS2 Jazzy。Jazzy 推荐⾛ Gazebo Harmonic /
ros_gz  路线，Launch、插件和 SDF 写法与本⽂的  gazebo_ros_pkgs  / Gazebo Classic 路线不同。等
Humble 版本跑通后，再迁移 Jazzy 会更稳。

## 2.2 必装软件包（含仿真）

```sh
export ROS_DISTRO=humble
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  ros-$ROS_DISTRO-tf2-tools \
  ros-$ROS_DISTRO-gazebo-ros-pkgs \
  python3-colcon-common-extensions \
  python3-rosdep
```
如果这台机器第⼀次⽤ ROS：
```sh
sudo rosdep init
rosdep update
```

## 2.3 安装后验证

```sh
ros2 pkg list | grep nav2
ros2 pkg list | grep slam_toolbox
ros2 pkg list | grep robot_localization
ros2 pkg list | grep gazebo_ros
rviz2 --help
```
期望结果：能看到  nav2_* 、 slam_toolbox 、 robot_localization 、 gazebo_ros  相关包，且
rviz2 --help  正常输出。

## 2.4 核⼼组件⼀览

组件                   | 作⽤
----------------------|---
robot_state_publisher | 根据URDF 发布机器⼈连杆和静态关节的 TF
gazebo_ros            | 在 gazebo 运⾏ ROS2 插件，提供仿真传感器和底盘动⼒
robot_localization    | 融合仿真⾥程计和仿真 IMU，发布odom -> base_link
slam_toolbox          | 建图阶段负责建图，同时提供map -> odom
map_server            | 导航阶段加载静态地图
amcl                  | 导航阶段基于静态地图定位，提供map -> odom
planner_server        | 计算全局路径
controller_server     | 计算局部速度控制
bt_navigator          | ⽤ Behavior Tree 组织导航流程
costmap               | 给规划器和控制器提供环境占据和障碍物表示

## 2.5 命令约定
 
本⽂⽤  $HOME/ros2humble_nav_ws  作为 Workspace 路径。建议在每个新终端先执⾏：

```sh
export ROS_DISTRO=humble
source /opt/ros/humble/setup.bash
export ROBOT_WS=$HOME/ros2humble_nav_ws
# export ROBOT_WS=$HOME/ros/ros2humble_nav_ws
```

⼯作空间构建完成后，再加：
```sh
source $ROBOT_WS/install/setup.bash
```



## 2.6 术语约定

为避免 ROS2 专业术语被直译后产⽣误解，本⽂保留以下英⽂术语：

术语            | 本⽂含义
---------------|-----
Workspace      | ROS2⼯程⼯作空间，例如$HOME/ros2humble_nav_ws
Package        | ROS2功能包
Node           | ROS2运⾏进程或组件节点
Topic          | ROS2发布/订阅通信通道
Message        | ROS2 Topic上传输的数据类型
Service        | ROS2 请求/响应接⼝
Frame          | TF坐标系名称，例如map、odom、base_link
Transform      | 两个Frame之间的位置和姿态关系
Parameter      | Node的运⾏参数
Launch         | ROS2启动描述⽂件
Costmap        | Nav2⽤于规划和避障的栅格代价地图
Lifecycle Node | 具有unconfigured / inactive / active 等状态的 ROS2 Node
BehaviorTree   | Nav2组织导航动作和恢复动作的⾏为树
rosbag         | ROS2数据记录⽂件

# 3. ⼯作空间

## 3.1 建议⽬录

```sh
ros2humble_nav_ws/
  src/
    robot_nav_description/
      urdf/
    robot_nav_bringup/
      launch/
      config/
      worlds/
    robot_nav_slam/
    robot_nav_tools/
  maps/            # 保存的地图
  bags/            # rosbag
```

## 3.2 每个 Package 的职责
* robot_nav_description ：URDF / xacro、机器⼈模型、传感器安装位姿、Gazebo 插件配置。
* robot_nav_bringup ：Launch ⽂件、Nav2 参数、 robot_localization  参数、导航启动。
* robot_nav_slam ：SLAM Toolbox 配置和建图阶段 Launch。
* robot_nav_tools ：调试脚本、记录 rosbag、健康检查⼯具。

## 3.3 创建最⼩⼯作空间

```sh
mkdir -p $ROBOT_WS/src
cd $ROBOT_WS
rosdep update
```

创建最⼩两个 Package：

```sh
cd $ROBOT_WS/src
ros2 pkg create robot_nav_description --build-type ament_cmake
ros2 pkg create robot_nav_bringup --build-type ament_cmake
```

## 3.4 构建与加载

```sh
cd $ROBOT_WS
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep robot_nav
```
 
# 4. 仿真环境搭建

## 4.1 Gazebo 仿真世界

在  robot_nav_bringup/worlds/  下创建⼀个最⼩仿真世界。这⾥提供⼀个 5m × 5m 的⾛廊+房间场
景，⾜够第⼀周使⽤：
```sh
mkdir -p $ROBOT_WS/src/robot_nav_bringup/worlds
```

创建  $ROBOT_WS/src/robot_nav_bringup/worlds/office.world ：


```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="office">
    <physics name="default" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- 墙壁示例：左墙 -->
    <model name="wall_left">
      <static>true</static>
      <pose>0 -2.5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>5 0.1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>5 0.1 1</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
    <!-- 墙壁示例：右墙 -->
    <model name="wall_right">
      <static>true</static>
      <pose>0 2.5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>5 0.1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>5 0.1 1</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
    <!-- 前墙 -->
    <model name="wall_front">
      <static>true</static>
      <pose>2.5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.1 5 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.1 5 1</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
    <!-- 后墙 -->
    <model name="wall_back">
      <static>true</static>
      <pose>-2.5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.1 5 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.1 5 1</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```
这只是示例。实际使⽤时，可以⽤ Gazebo ⾃带的  building_editor  画⼀个更丰富的室内环境，或者
从⽹上下载  .world  ⽂件。

## 4.2 仿真世界的验收

启动 Gazebo 并加载世界：
```sh
# `gazebo_ros`（gazebo_ros_pkgs）是 Gazebo Classic（老 Gazebo，gazebo11）的包
# --------------
# ros2 launch gazebo_ros gazebo.launch.py \
#   world:=$HOME/ros2humble_nav_ws/src/robot_nav_bringup/worlds/office.world

ros2 launch gazebo_ros gazebo.launch.py \
  world:=$HOME/ros/ros2humble_nav_ws/src/robot_nav_bringup/worlds/office.world.sdf  
```
期望结果：
Gazebo 正常打开，能看到地⾯和墙壁。
没有报错。

如果这⾥失败，不要继续。先检查  GAZEBO_MODEL_PATH  或重新安装  gazebo_ros_pkgs 。

# 5. 仿真机器⼈模型与 URDF

## 5.1 仿真模型需要包含什么

在  robot_nav_description  中建⽴  xacro  模型，⾄少包含：

* base_link （带碰撞体和惯性参数，Gazebo 需要）
* 第⼀版建议⽤“左右两个驱动轮 + 两个从动轮/脚轮”的差速模型（带  gazebo_ros_diff_drive  插件）
* laser （带  gazebo_ros_ray  或  gazebo_ros_laser  插件）
* imu_link （带  gazebo_ros_imu  插件）
* gps_link （预留，暂不启⽤）

## 5.2 坐标⽅向约定

ROS 机器⼈常⽤约定：

* x  轴朝机器⼈前⽅。
* y  轴朝机器⼈左侧。
* z  轴朝上。

在仿真⾥，这个约定直接决定了  /scan  点云的⽅向和  /wheel/odom  的符号。先统⼀，再写 URDF。

## 5.3 Gazebo 插件配置示例

下⾯给出仿真传感器插件的关键⽚段（需要嵌⼊到完整的 xacro 中）。第⼀周建议先⽤“两驱差速”简化模
型：左、右各⼀个驱动轮，另外两个轮⼦作为从动轮或视觉模型。这样 Nav2、SLAM、EKF 的主链路能先
跑通，避免⼀开始就被四轮滑移转向动⼒学卡住。

如果真实机器⼈是四轮差速/滑移转向，第⼆阶段再切换到  ros2_control  或专⻔的 skid-steer 控制插
件，不建议在第⼀天就把复杂动⼒学和导航链路混在⼀起调。

### 差速底盘驱动插件：
```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/</namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=wheel/odom</remapping>
    </ros>
    <update_rate>50</update_rate>
    <left_joint>front_left_wheel_joint</left_joint>
    <right_joint>front_right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>false</publish_odom_tf>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>
</gazebo>
```
 

注意： publish_odom_tf  设为  false ，因为  odom -> base_link  由  robot_localization  发布，
避免冲突。

### 关节状态插件：
<!-- claude code added -->
```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <namespace>/</namespace>
    </ros>
    <update_rate>30</update_rate>
    <joint_name>front_left_wheel_joint</joint_name>
    <joint_name>front_right_wheel_joint</joint_name>
  </plugin>
</gazebo>
```

驱动轮关节建议写  type="continuous" （连续旋转、没有⾓度上下限）：

```xml
<joint name="front_left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="front_left_wheel"/>
  <origin xyz="0.15 0.2 0.0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit effort="1000" velocity="10"/>
</joint>
```

为什么这两条必须⼀起出现：驱动轮是  continuous  /  revolute  这类可动关节，它们的 TF 不由
URDF 静态发布，必须由  robot_state_publisher  从  /joint_states  读取后再发布。只在 URDF ⾥
写了轮⼦、却没加关节状态插件，RViz 的 RobotModel 会报：

```txt
No transform from [front_left_wheel] to [map]
No transform from [front_right_wheel] to [map]
```

注意：写  revolute  也可以，但  <limit>  必须同时给出  lower / upper ，否则会被当成 0，轮⼦有
被锁死的⻛险。轮⼦是连续旋转的，直接⽤  continuous  更合适。

注意：仿真⾥⽤这个插件发布  /joint_states  之后，不要再  ros2 run joint_state_publisher ，
否则同⼀个话题会有两个发布者。

## 激光雷达插件：
```xml
<gazebo reference="laser">
  <sensor name="laser" type="ray">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>20</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="laser" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>laser</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

IMU 插件：
```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <plugin name="imu" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <output_type>sensor_msgs/Imu</output_type>
      <frame_name>imu_link</frame_name>
      <update_rate>100</update_rate>
    </plugin>
  </sensor>
</gazebo>
```

## 5.4 URDF 与 Gazebo 的衔接

如果传感器和机身关系固定，写进 URDF，让  robot_state_publisher  统⼀发布 TF。
Gazebo 插件也写在 URDF 的  <gazebo>  标签中，仿真启动时⾃动加载。
真机运⾏时， robot_state_publisher  通常会忽略  <gazebo>  标签，但⼯程上不建议⻓期把仿真插
件⽆条件混在真机模型⾥。更推荐在 xacro 中加开关：

```xml
<xacro:arg name="use_gazebo" default="false"/>
<xacro:if value="$(arg use_gazebo)">
  <!-- Gazebo sensors and drive plugins here -->
</xacro:if>
```
仿真 Launch 使⽤  use_gazebo:=true ，真机 Launch 使⽤  use_gazebo:=false 。这样同⼀份 xacro
可以复⽤⼏何和 TF，⼜不会让仿真插件污染真机配置。

<!-- 

## 5.5. 话题输出

- `/cmd_vel`：速度指令输入
- `/wheel/odom`：轮式里程计输出
- `/scan`：激光雷达
- `/imu/data`：IMU 数据

> 
> odom → base_link 的 tf 交给 robot_localization EKF，不会和 gazebo 插件冲突。

## 5.6 可选后续修改点

1. 调整各部件 xyz 位置匹配真实机器人；
2. 修改质量、惯性参数让 Gazebo 动力学更真实；
3. 后续需要 GPS 仿真时，在`<xacro:if>`块内增加 gazebo gps 传感器插件；
4. 后期切换四轮滑移转向时，替换 diff_drive 插件。

如果你需要，我可以顺便写一个 `display.launch.py` + `gazebo.launch.py` 来加载这个 xacro 模型。
 -->

# 6. 接⼝约定

## 6.1 仿真阶段必须统⼀的 Topic

功能      Topic               MessageType            来源
---------|-------------------|-----------------------|--------------------------
激光雷达   | /scan             | sensor_msgs/LaserScan |  Gazebo 插件
轮速⾥程计 | /wheel/odom       | nav_msgs/Odometry     |  gazebo_ros_diff_drive
IMU      | /imu/data         | sensor_msgs/Imu       |  Gazebo 插件
速度指令  | /cmd_vel           | geometry_msgs/Twist   |  键盘遥控 / Nav2
融合⾥程计| /odometry/filtered | nav_msgs/Odometry      | robot_localization
地图     | /map               | nav_msgs/OccupancyGrid | slam_toolbox/map_server

## 6.2 必须统⼀的 Frame
```sh
map -> odom -> base_link -> laser
                         -> imu_link
                         -> gps_link
```

## 6.3 TF 发布职责对照表

阶段   | Transform              | Publisher          |   备注
-------|-----------------------|--------------------|-------------------------
建图    | odom -> base_link     | robot_localization |   融合仿真⾥程计 + 仿真 IMU       
建图    | map -> odom           | slam_toolbox       |   同时承担建图和定位  
导航    | odom -> base_link     | robot_localization |   与建图阶段相同
导航    | map -> odom           | amcl               |   基于静态地图定位
任意阶段 | base_link -> laser    | robot_state_publisher | 来⾃ URDF 
任意阶段 | base_link -> imu_link | robot_state_publisher | 来⾃ URDF

# 7. 仿真数据检查
仿真数据是“完美的”，但检查流程和真机完全⼀样。第⼀周的核⼼任务是学会这套检查⽅法。

## 7.1 激光雷达
```sh
ros2 topic echo /scan --once
ros2 topic hz   /scan
```

重点看：
* header.frame_id  是否为  laser 。
* range_min 、 range_max  是否与 Gazebo 插件配置⼀致。
* 数据是否全 0、全  inf 。

## 7.2 轮速⾥程计
```sh
ros2 topic echo /wheel/odom --once
ros2 topic hz  /wheel/odom
```

重点看：
* 在 Gazebo 中向前推机器⼈时  x  是否增加。
* 左转时 yaw 是否按 ROS 约定变化（逆时针为正）。
* covariance  是否不是全 0（Gazebo 插件通常会填充⼀个默认值）。

## 7.3 IMU
```sh
ros2 topic echo /imu/data --once
ros2 topic hz  /imu/data
```

重点看：
* frame_id  是否为  imu_link 。
* 静⽌时⻆速度是否接近 0。
* 机器⼈左转时 yaw 变化⽅向是否符合预期。

## 7.4 传感器通关检查表

进⼊ SLAM 前，⾄少满⾜：

检查项          | 最低标准
---------------|-------------------------------------------
/scan          | 频率稳定， frame_id  正确，不是全 0 / 全  inf
/wheel/odom    | 前进⽅向正确，转向⽅向正确，速度量级合理
/imu/data      | 静⽌时⻆速度接近 0，转向⽅向正确
TF             | base_link -> laser  和  base_link -> imu_link  可查到
  

# 8. 状态估计：robot_localization

## 8.1 EKF 最⼩可⽤配置
```yaml
# robot_localization 参数⽂件（2D EKF，仿真版）
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.2
    two_d_mode: true
    publish_tf: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /wheel/odom
    odom0_config: [true,  true,  false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false

    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: true
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true
```

## 8.2  odom0_config  数组含义

顺序固定为 15 个布尔值： [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw,
ax, ay, az] 。 true  表示该维度参与融合。

## 8.3  world_frame: odom  的含义

这个 EKF 发布的是  odom -> base_link ，不是  map -> odom 。

# 9. Nav2 参数基线

## 9.1 先复制官⽅可运⾏配置

不要从零⼿写整份  nav2_params.yaml 。Nav2 的完整参数包含  map_server 、 amcl 、
planner_server 、 controller_server 、 behavior_server 、 bt_navigator 、
waypoint_follower 、 velocity_smoother 、 lifecycle_manager 、 local_costmap 、
global_costmap  等多组 Node。新⼿⼿写容易漏项，导致 Lifecycle Node ⽆法进⼊  active 。

建议先复制 Humble ⾃带的默认参数，再按本⽂修改关键项：
```sh
mkdir -p $ROBOT_WS/src/robot_nav_bringup/config
cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
  $ROBOT_WS/src/robot_nav_bringup/config/nav2_params.yaml
```

后续所有导航启动命令都必须显式传⼊这份⽂件：
```sh
params_file:=$ROBOT_WS/src/robot_nav_bringup/config/nav2_params.yaml
```
如果忘记传  params_file ，Nav2 可能会使⽤默认参数，导致“我明明改了参数但⾏为没变”的假象。

## 9.2 仿真阶段必须修改的关键项

在复制出来的  nav2_params.yaml  中检查并修改这些参数：
```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    base_frame_id: base_link
    odom_frame_id: odom
    global_frame_id: map
    scan_topic: /scan
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.10
      movement_time_allowance: 10.0
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.30
      yaw_goal_tolerance: 0.50
      stateful: true
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.15
      max_vel_theta: 0.35
      acc_lim_x: 0.25
      acc_lim_theta: 0.50
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 10.0 
      publish_frequency: 5.0
      rolling_window: true
      width: 4.0
      height: 4.0
      resolution: 0.05
      robot_radius: 0.30
      obstacle_layer:
        scan:
          topic: /scan
          data_type: LaserScan
          clearing: true
          marking: true
          obstacle_max_range: 4.0
          raytrace_max_range: 5.0
      inflation_layer:
        inflation_radius: 0.70
        cost_scaling_factor: 3.0
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      robot_radius: 0.30
      obstacle_layer:
        scan:
          topic: /scan
          data_type: LaserScan
          clearing: true
          marking: true
      inflation_layer:
        inflation_radius: 0.70
        cost_scaling_factor: 3.0
```
这段 YAML 是关键修改摘录，不是完整⽂件。完整⽂件应保留官⽅默认配置⾥的其他 Node 和 plugin 设置。

## 9.3 全局检查

仿真阶段，整份  nav2_params.yaml  中所有  use_sim_time  都应为  true ：
```sh
grep -n "use_sim_time" $ROBOT_WS/src/robot_nav_bringup/config/nav2_params.yaml
```

## 9.4  robot_radius  和  footprint  怎么选

如果机器⼈近似圆形，第⼀版先⽤  robot_radius ：
```sh 
robot_radius: 0.30
```

如果机器⼈明显是矩形，改⽤  footprint ：
```sh
footprint: "[[0.30, 0.20], [0.30, -0.20], [-0.30, -0.20], [-0.30, 0.20]]"
```
设置了  footprint  后建议删除  robot_radius 。

# 10. Launch ⽂件与 SLAM 建图（仿真）

## 10.1 创建  simulation.launch.py

在  robot_nav_bringup/launch/  下创建  simulation.launch.py 。这个 Launch 负责⼀次性启动
Gazebo、机器⼈模型、TF 和 EKF：
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('robot_nav_bringup'),
        'worlds',
        'office.world',
    ])

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('robot_nav_description'),
            'urdf',
            'robot.urdf.xacro',
        ]),
        ' use_gazebo:=true',
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])
        ]),
        launch_arguments={'world': world}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'nav_robot'],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_nav_bringup'),
                'config',
                'ekf.yaml',
            ]),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ekf,
    ])
```

如果  worlds/office.world  放在 Workspace 根⽬录⽽不是 Package 内，
FindPackageShare('robot_nav_bringup')  找不到它。建议把 world ⽂件也放进
robot_nav_bringup/worlds/ ，并在  CMakeLists.txt  安装：
```cmake
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)
```

每次新增 Launch / config / worlds 后重新构建：
```sh
cd $ROBOT_WS
colcon build --symlink-install
source install/setup.bash
```

## 10.2 启动仿真机器⼈链路
```sh
ros2 launch robot_nav_bringup simulation.launch.py
```
这个阶段先不启动 SLAM。先检查  /scan 、 /wheel/odom 、 /imu/data 、 /odometry/filtered  和
TF 是否正常。

## 10.3 启动 SLAM Toolbox

### 10.3.1 先创建  slam_params.yaml
<!-- claude code added -->
slam_toolbox 的默认  base_frame  是  base_footprint ，但本机器人的 URDF 根 link 是
base_link ，全工程（URDF / ekf.yaml / nav2_params.yaml，见 6.2、8.1、9.2）统一使⽤
map -> odom -> base_link 。如果直接按默认参数启动，会看到刷屏：

```txt
[WARN] [slam_toolbox]: Failed to compute odom pose
[INFO] [slam_toolbox]: Message Filter dropping message: frame 'laser' ... queue is full
```

原因是 slam_toolbox 在找一个本机器人根本不存在的  base_footprint 。所以在
robot_nav_bringup/config/  下创建  slam_params.yaml ，只覆盖坐标系相关的项：

```yaml
# slam_toolbox 参数文件（仿真，建图阶段）
# 只覆盖与默认值不同的项，其余调参保持节点默认值
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_link     # 默认是 base_footprint，必须改成本机器人的 base_link
    scan_topic: /scan
    mode: mapping             # 建图⽤ mapping；加载已有地图定位时改 localization
```

注意：这⾥故意不写  use_sim_time ，它由启动命令传⼊（和 9.1 中  params_file  的道理
⼀样），否则第⼆周迁移真机时容易漏改。

后期如果要把参数摊开调（例如 10.5 中地图撕裂时要动  minimum_travel_distance 、
minimum_travel_heading ），可以按 9.1 的做法先复制官⽅默认再改：

```sh
cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml \
  $ROBOT_WS/src/robot_nav_bringup/config/slam_params.yaml
# 然后把 base_frame 改成 base_link
```

### 10.3.2 启动
<!-- claude code added -->
config  下新增⽂件后要先重新构建（install(DIRECTORY ...)  只在上次构建时建符号链接）：

```sh
cd $ROBOT_WS
colcon build --symlink-install
source install/setup.bash
```

然后启动：

```sh
ROBOT_WS=/root/ros/ros2humble_nav_ws
# terminal 01
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=$ROBOT_WS/src/robot_nav_bringup/config/slam_params.yaml
# terminal 02 
rviz2 --ros-args -p use_sim_time:=true  
```

注意：Humble 的参数⽂件参数名是  slam_params_file （不是 Foxy 时代的  params_file ）。
名字写错不会报错，节点会静默退回默认参数⽂件，于是⼜变回  base_footprint 。
不确定时先⽤下⾯的命令确认：

```sh
ros2 launch slam_toolbox online_async_launch.py --show-args
```

启动后确认参数真的⽣效、且不再刷  Failed to compute odom pose ：

```sh
ros2 param get /slam_toolbox base_frame
# 期望结果：String value is: base_link
```

## 10.4 ⽤键盘移动机器⼈
```sh
# terminal 03
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
在仿真中低速直⾏、低速转弯、绕闭环。观察 RViz 中地图是否稳定展开。

## 10.5 建图阶段的现象

正常：
* 激光扇形跟随机器⼈移动。
* 地图边⾛边⻓出来。
* 绕回起点时，地图不会明显裂开成两层。
* 机器⼈停下时，位姿不持续漂移。

异常：
* 激光扇形在机器⼈后⾯（ base_link  前向定义错误）。
* 机器⼈向前⾛，地图反着移动（ odom  ⽅向错误）。
* 原地慢转时地图整体撕裂（ slam_toolbox  参数或  odom  质量不够）。

# 11. 保存地图
```sh
# terminal 03
mkdir -p $HOME/ros2humble_nav_ws/maps
ros2 run nav2_map_server map_saver_cli -f $HOME/ros2humble_nav_ws/maps/demo_map
# ros2 run nav2_map_server map_saver_cli -f /root/ros/ros2humble_nav_ws/maps/demo_map
```
⽣成  demo_map.yaml  和  demo_map.pgm 。

# 12. 切换到导航模式（仿真）

## 12.1 切换逻辑

* 停⽌  slam_toolbox
* 保留 Gazebo、机器⼈、TF、 robot_localization
* 启动  map_server
* 启动  amcl
* 启动 Nav2 其他服务器

<!-- 
```sh 
<!-- abner-added 为了方便,把导航模式的所有步骤放到这里-->
# docker terminal 01
root@a:~/ros/ros2humble_nav_ws# ros2 launch robot_nav_bringup simulation.launch.py
# docker terminal 02 
root@a:~/ros/ros2humble_nav_ws# ros2 launch nav2_bringup bringup_launch.py   map:=/root/ros/ros2humble_nav_ws/maps/demo_map.yaml   use_sim_time:=true   params_file:=/root/ros/ros2humble_nav_ws/src/robot_nav_bringup/config/nav2_params.yaml
# docker terminal 03
root@a:~/ros/ros2humble_nav_ws# rviz2 --ros-args -p use_sim_time:=true
``` 
-->
## 12.2 Lifecycle Node 概念

Nav2 核⼼服务器是 ROS2 Lifecycle Node，需要经历  unconfigured -> inactive -> active 。
bringup_launch.py  会⾃动完成。⼿动排查时可⽤：
```sh
ros2 lifecycle get /controller_server
ros2 lifecycle set /controller_server activate
```

## 12.3 Behavior Tree 是什么

默认  NavigateToPose  是 Behavior Tree 流程：
```txt
ComputePathToPose -> FollowPath
如果失败 -> Recovery（清理 Costmap / 原地旋转 / 后退 / 等待）
```
如果看到机器⼈原地旋转，先判断这是 recovery 还是故障。

## 12.4 导航模式启动
```sh
# ros2 launch nav2_bringup bringup_launch.py \
#   map:=$HOME/ros2humble_nav_ws/maps/demo_map.yaml \
#   use_sim_time:=true \
#   params_file:=$HOME/ros2humble_nav_ws/src/robot_nav_bringup/config/nav2_params.yaml

# ----
ros2 launch nav2_bringup bringup_launch.py \
  map:=/root/ros/ros2humble_nav_ws/maps/demo_map.yaml \
  use_sim_time:=true \
  params_file:=/root/ros/ros2humble_nav_ws/src/robot_nav_bringup/config/nav2_params.yaml  

rviz2 --ros-args -p use_sim_time:=true  
```

启动后检查你修改过的 Parameter 是否真的⽣效：
```sh
ros2 param dump /controller_server | grep -E "max_vel_x|max_vel_theta"
ros2 param dump /local_costmap/local_costmap | grep -E "robot_radius|inflation_radius"
```

## 12.5 RViz 操作

1. ⽤  2D Pose Estimate  给出初始位姿。
2. 等待  amcl  收敛。
3. ⽤  Nav2 Goal  发送⽬标点。

# 13. 第⼀个完整⽤例（仿真）

## 13.1 ⽤例名称

NAV-CASE-001-SIM：仿真室内建图与定点导航


## 13.2 前置条件

* Gazebo 世界正常加载
* /scan 、 /wheel/odom 、 /imu/data  可⻅且频率稳定
* TF 树完整
* 键盘遥控可⽤

## 13.3 步骤

1. 启动仿真环境和机器⼈。
2. 启动  robot_localization 。
3. 启动  slam_toolbox 。
4. ⽤键盘移动机器⼈完成建图。
5. 保存地图。
6. 停⽌  slam_toolbox 。
7. 启动  map_server + amcl + Nav2 。
8. 在 RViz 中设置初始位姿。
9. 发送 2-5 ⽶导航⽬标。
0. 观察路径规划与执⾏。

## 13.4 通过标准

* 地图保存成功。
* amcl  和 Nav2 Node 进⼊  active  状态。
* 能⽣成全局路径。
* 机器⼈低速避障到达⽬标。
* 没有碰撞。
* ⽬标误差在⾸版容差范围内。

# 14. 调试与诊断（仿真）

## 14.1 最⼩调试闭环

1.  ros2 topic list
2.  ros2 topic hz /scan
3.  ros2 topic echo /scan --once
4.  ros2 topic echo /wheel/odom --once
5.  ros2 topic echo /imu/data --once
6.  ros2 run tf2_tools view_frames
7. 启动  slam_toolbox
8. 打开 RViz 看地图和激光
9. 保存地图
0. 启动导航模式
1. 设置初始位姿
2. 发第⼀个⽬标

## 14.2 TF 诊断
```sh
ros2 run tf2_tools view_frames
```

## 14.3 Lifecycle 状态
```sh
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

## 14.4 查看实际参数
```sh
ros2 param dump /controller_server
```

## 14.5 清理局部 Costmap
```sh
ros2 service call /local_costmap/clear_entirely_local_costmap \
  nav2_msgs/srv/ClearEntireCostmap "{}"
```

# 15. 常⻅故障（仿真）

## 15.1 没有地图
```sh
slam_toolbox  是否启动
/map  是否有数据
RViz Fixed Frame 是否为  map
use_sim_time  是否⼀致
```

## 15.2 Nav2 不动
```sh
/cmd_vel  是否有输出
Gazebo 中的机器⼈是否真的订阅了  /cmd_vel
Lifecycle Node 是否  active
Costmap 是否把机器⼈包在障碍⾥
```

## 15.3 原地转圈

先判断 recovery 还是故障。如果怀疑故障：

* 初始位姿错
* TF 延迟
* yaw_goal_tolerance  太⼩

## 15.4 贴墙或撞障碍

* robot_radius  或  footprint
* inflation_radius
* cost_scaling_factor

# 16. 仿真阶段交付物

完成第⼀周时，⾄少交付：
* ⼀个可构建的 ROS2 workspace
* ⼀份带 Gazebo 插件的机器⼈ URDF / xacro
* ⼀份  robot_localization  参数⽂件（ ekf.yaml ）
* ⼀份 Nav2 参数⽂件（ nav2_params.yaml ）
* ⼀份仿真 Launch ⽂件（ simulation.launch.py ）
* ⼀张仿真地图（ .yaml + .pgm ）
* ⼀次成功的仿真定点导航（附 RViz 截图或 rosbag）
* ⼀份参数说明⽂档（记录哪些参数是仿真⾥验证的，哪些需要真机微调）

这些⽂件在第⼆周可以复⽤结构和命名约定，但真机上必须重新验证数值。⾄少要重新确认
use_sim_time=false 、传感器 Frame、协⽅差、速度上限、加速度上限、 footprint  /
robot_radius  和  inflation_radius 。

# 17. 仿真 → 真机迁移清单（预览）

第⼆周开始前，确认以下事项：

仿真项         | 真机迁移⽅式
--------------|----------------
use_sim_time  | true  →  false
Gazebo 世界    | 替换为真实场地
gazebo_ros_diff_drive | 替换为真实底盘驱动（发布  /wheel/odom ）
Gazebo 激光插件         | 替换为真实雷达驱动（发布  /scan ）
Gazebo IMU 插件        | 替换为真实 IMU 驱动（发布  /imu/data ）
URDF / xacro          | 复⽤结构，尺⼨、传感器安装位姿和  use_gazebo  开关按真机重核
robot_localization    | 参数 复⽤融合策略，协⽅差、频率、延迟和 IMU ⽅向按真机重核
Nav2 参数              | 复⽤参数⽂件结构，速度上限、加速度、 footprint 、膨胀半径按真机重核

# 18. 延伸阅读

## Nav2 First-Time Robot Setup Guide

https://docs.nav2.org/setup_guides/index.html
重点看：Gazebo 仿真相关章节、TF、URDF、EKF。

## Nav2 SLAM 教程

https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html

## Nav2 Costmap 2D

https://docs.nav2.org/configuration/packages/configuring-costmaps.html

# 附录：如何使⽤这份⼿册

这份⼿册不是让你⼀次性读完再动⼿。推荐节奏是：

1. 读⼀⼩节。
2. ⽴刻执⾏对应命令或检查。
3. 对照“期望结果”判断是否通过。
4. 没通过就停下来修，不要继续往下叠新东⻄。

仿真阶段的⽬标是建⽴信⼼、熟悉⼯具链、验证参数结构。不要追求“⼀次调完美”，先把闭环跑起来，第
⼆周上真机时你会感谢第⼀周的积累。

