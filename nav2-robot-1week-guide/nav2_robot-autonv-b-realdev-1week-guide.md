<<Nav2 机器⼈⾃主导航（下）：ROS2 + SLAM + EKF真机⼀周指南>>

⾯向对象：已完成《Nav2 机器⼈⾃主导航（上）》的仿真闭环，或在其他仿真环境中已验证过 ROS2 +
Nav2 + SLAM + EKF 链路，现在准备把这套系统迁移到真实机器⼈上的软件⼯程师。

⽬标：把经过仿真验证的参数和流程，迁移到真实硬件，完成第⼀个“真机建图 → 保存地图 → 加载地图 →
设置初始位姿 → 发送导航⽬标”的闭环。

这份⽂档的核⼼假设是：你⼿⾥已经有⼀套在仿真⾥跑通的参数⽂件（ robot_localization  YAML +
nav2_params.yaml ）和 URDF 结构。第⼆周的任务不是重新发明这些参数，⽽是替换硬件驱动、校准
传感器⽅向、根据真机数据重新确认协⽅差和速度上限、解决仿真⾥遇不到的硬件问题。

# 1. ⽬标与边界

## 1.1 第⼆周⽬标（真机）

* 把仿真中验证过的 URDF、EKF 参数、Nav2 参数迁移到真实机器⼈。
* 接⼊真实底盘驱动、2D 激光雷达、IMU，统⼀ Topic 命名。
* 完成真实传感器的⽅向验证、频率检查、数据质量⻔槛。
* 在真实场地中完成 SLAM 建图、保存地图、Nav2 定点导航。
* ⽣成⼀套经过真机验证的完整参数包，作为后续迭代的基准。

## 1.2 第⼆周不做什么

不重新设计 URDF 结构（复⽤仿真版结构，但按真机重测尺⼨和安装位姿）。
不重新推导 EKF 融合策略（复⽤仿真版策略，但按真机重核协⽅差、频率、延迟和⽅向）。
不重新学习 Nav2 架构（复⽤仿真版参数结构，但按真机重核速度、加速度、footprint 和 Costmap）。
不做室外导航。
不做⾼速运⾏。
不做视觉、语义、多传感器融合。
不做⾃动充电、巡检业务。

## 1.3 第⼀周与第⼆周的关系

第⼀周（仿真）                    第⼆周（真机）
─────────────────────────────────────────────────────
Gazebo 世界          ──────→    真实场地
仿真底盘插件          ──────→    真实底盘驱动
仿真雷达插件          ──────→    真实雷达驱动
仿真 IMU 插件         ──────→    真实 IMU 驱动
URDF（示范尺⼨）      ──────→    URDF（真实尺⼨）
EKF 参数（仿真基线）  ──────→    EKF 参数（真机重核）
Nav2 参数（保守基线） ──────→    Nav2 参数（真机重核）
use_sim_time=true    ──────→    use_sim_time=false

# 2. 软件栈

## 2.1 操作系统和 ROS2

与仿真相同，不需要额外安装新的⼤版本：
* Ubuntu 22.04 + ROS2 Humble

如果第⼀周已经按 Humble 跑通，第⼆周尽量继续⽤同⼀套环境，不要在真机阶段再切到另⼀个 ROS2 发
⾏版。

## 2.2 必装软件包（真机，不含 Gazebo）
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
  python3-colcon-common-extensions \
  python3-rosdep
```
不需要安装  gazebo_ros_pkgs （除⾮你想同时保留仿真环境做对⽐）。


## 2.3 ⼯作空间复⽤

直接复⽤第⼀周的  robot_nav_ws ：
```sh
cd $HOME/robot_nav_ws
colcon build --symlink-install
source install/setup.bash
```
如果第⼀周已经构建过，通常只需要重新  source 。真机阶段建议保留同⼀份 Workspace，新增
robot_nav_hardware  或  robot_nav_real  Package，⽽不是把仿真和真机配置混在⼀个⽂件⾥⼿
改。

## 2.4 真机安全⻔槛

在让 Nav2 ⾃动控制底盘之前，必须逐项确认：

> 1. 急停有效，且操作员知道如何⽴即断开运动。
> 2. 轮⼦悬空或低速空载测试已通过。
> 3. ⼿动遥控可以稳定前进、后退、左转、右转。
> 4.  /cmd_vel  的订阅对象是你真实底盘。
> 5. 底盘速度上限已在驱动层和 Nav2 层同时限制。
> 6. 测试区域⾜够空旷，旁边有⼈看护。

# 3. 真机硬件接⼊

## 3.1 接⼊顺序

> 1. 底盘驱动（差速轮式，发布  /wheel/odom ）
> 2. 2D 激光雷达（发布  /scan ）
> 3. IMU（发布  /imu/data ）
> 4. 急停开关（硬件安全，确保可⽤）

## 3.2 驱动接⼊原则

* 如果底盘⼚商提供了 ROS2 驱动，直接使⽤。
* 如果只有串⼝ / CAN 协议，需要⾃⼰写最⼩驱动 Node，把原始数据封装成  nav_msgs/Odometry  和geometry_msgs/Twist 。
* 如果雷达⼚商提供了 ROS2 驱动，直接使⽤。
* 如果只有原始扫描数据，需要⾃⼰封装成  sensor_msgs/LaserScan 。

## 3.3 Topic 统⼀

⽆论驱动原本发布什么名字，都要统⼀为：


功能       | Topic       |  Message Type
----------|---------------------
激光雷达    | /scan       | sensor_msgs/LaserScan
轮速⾥程计  | /wheel/odom | nav_msgs/Odometry
IMU        | /imu/data  | sensor_msgs/Imu
速度指令    | /cmd_vel    | geometry_msgs/Twist
 
 

如果驱动⾃带的名字不同，⽤  remap ：
```py
# 在 launch ⽂件中
Node(
    package='some_driver',
    executable='driver_node',
    remappings=[
        ('/some_driver/scan', '/scan'),
        ('/some_driver/odom', '/wheel/odom'),
    ]
)
```

## 3.4 硬件接⼊验收
```sh
ros2 topic list | grep -E "scan|odom|imu|cmd_vel"
ros2 topic hz /scan
ros2 topic hz /wheel/odom
ros2 topic hz /imu/data
```
期望：四个 Topic 都存在，频率稳定（雷达 10–20 Hz，⾥程计 20–50 Hz，IMU 50–100 Hz）。

# 4. 数据质量⻔槛

这是真机阶段最重要的⻔槛。仿真数据是“完美的”，真机数据往往有噪声、延迟、⽅向错误。先把数据质
量确认清楚，再启动 SLAM。

## 4.1 激光雷达
```sh
ros2 topic echo /scan --once
ros2 topic hz /scan
```

重点看：
* header.frame_id  是否真的是  laser  或约定的名字。
* range_min 、 range_max  是否合理。
* 数据是否全 0、全  inf 、⼤⾯积  NaN 。
* 雷达频率是否稳定（不应剧烈波动）。
* 雷达安装位置是否被机器⼈本体遮挡（检查最近⼏个 scan 点是否全是 0）。

经验提醒：不同雷达驱动对超出量程数据的处理⽅式不同（ 0 、 inf  或  NaN ）。建图前先知道你的驱动
⻓什么样，后⾯ Costmap 异常时才不会盲查。

## 4.2 轮速⾥程计
```sh
ros2 topic echo /wheel/odom --once
ros2 topic hz /wheel/odom
```

重点看：
* 向前推机器⼈时  x  是否增加。
* 左转时 yaw 是否按 ROS 约定变化（逆时针为正）。
* 速度值是否和实际速度量级接近（例如实际 0.2 m/s 时，数据也应约为 0.2）。
* covariance  是否不是全 0。

为什么看 covariance： robot_localization  ⽤协⽅差决定融合权重。如果协⽅差全 0，系统会把该观
测当作“极度确定”，这在实际⼯程⾥经常不合理。如果底盘驱动没填协⽅差，建议先补⼀个合理的初始值
（例如位置协⽅差  0.1 ，速度协⽅差  0.01 ），⽽不是全 0。

## 4.3 IMU
```sh
ros2 topic echo /imu/data --once
ros2 topic hz /imu/data
```

重点看：
* frame_id  是否与 TF 命名⼀致。
* 静⽌时⻆速度是否接近 0（不应有持续⼏ rad/s 的漂移）。
* 机器⼈左转时 yaw 变化⽅向是否符合预期。
* IMU 是否已经做过基础标定（零偏、标度因数）。

如果 IMU 没标定，建议先做⼀个简单静态标定：让机器⼈静⽌ 30 秒，记录⻆速度平均值，作为零偏补偿
值。

## 4.4 传感器通关检查表

进⼊ SLAM 前，⾄少满⾜：

检查项       | 最低标准
------------|-----------
/scan       | 频率稳定， frame_id  正确，不是全 0 / 全  inf  / 全  NaN ，⽆本体遮挡
/wheel/odom | 前进⽅向正确，转向⽅向正确，速度量级合理， covariance  不全 0
/imu/data   | 静⽌时⻆速度接近 0，转向⽅向正确， frame_id  与 TF ⼀致
TF          | base_link -> laser  和  base_link -> imu_link  可查到
时间戳       | 各 Topic 时间戳持续更新，没有明显停滞或倒流

这张表没有通过时，不建议启动 SLAM。否则 SLAM 地图看似⽣成了，后⾯导航阶段会把问题放⼤。

# 5. 真机 URDF 与 TF

## 5.1 复⽤仿真 URDF

直接复制第⼀周的  robot_nav_description  中的 xacro ⽂件，然后修改：

* base_link  尺⼨改为真实测量值。
* 轮⼦相对位置改为真实测量值。
* 雷达安装  x/y/z  偏移改为真实测量值。
* IMU 安装  x/y/z  偏移和旋转改为真实测量值。
* 机器⼈前向定义确认⽆误。

Gazebo 插件部分不建议在真机 Launch 中启⽤。推荐在 xacro 中加  use_gazebo  开关：

```xml
<xacro:arg name="use_gazebo" default="false"/>
<xacro:if value="$(arg use_gazebo)">
  <!-- Gazebo drive and sensor plugins -->
</xacro:if>
```
真机 Launch 中使⽤  use_gazebo:=false 。这样可以复⽤机身⼏何、传感器安装位姿和 TF，同时避免
仿真插件配置混进真机运⾏环境。

## 5.2 坐标⽅向验证

在机器⼈底盘上贴三张⼩标签，标出  x / y / z 。在雷达和 IMU 上也标出它们各⾃的前向。测量传感器
相对  base_link  的偏移，再写 URDF，不要凭视觉估计。

期望结果：RViz 中 RobotModel 朝向和真实机器⼈⼀致， /scan  的激光扇形出现在机器⼈前⽅。

## 5.3 static_transform_publisher 过渡

如果 URDF 还没最终确定，可以先⽤  static_transform_publisher  临时验证： 
```sh
ros2 run tf2_ros static_transform_publisher \
  0.25 0.0 0.20 0 0 0 base_link laser
```
但这只是过渡⽅案。⻓期建议还是进 URDF。

# 6. 状态估计：robot_localization（真机微调）

## 6.1 复⽤仿真融合策略

直接把第⼀周验证过的  ekf.yaml  复制到真机 Package 中，作为真机调试起点。真机阶段重点不是先调
Nav2，⽽是先确认  /odometry/filtered  平滑、⽅向正确、时间戳连续。

## 6.2 真机需要微调的地⽅

仿真值   | 真机调整⽅向  | 原因
--------|--------------|
odom0_config中 x, y为true  | 通常保持true  | 轮速提供平⾯位置
odom0_config中 yaw为false  | 通常保持false | 避免与 IMU 冲突
imu0_config中yaw为true   | 如果IMU yaw很差，可改为false | 差 IMU 会引⼊漂移 
imu0_differential: true | 通常保持 true               | 避免 IMU 绝对 yaw 与轮速冲突
frequency: 50.0     |  如果融合后延迟明显，可提⾼到 100.0   |  真机数据可能更密集
sensor_timeout: 0.2 |  如果某个传感器丢包严重，适当增⼤      |  避免 EKF 因单帧丢失就发散

## 6.3 真机漂移排查

如果融合后位姿漂移很⼤，按这个顺序排查：

> 1. 先查  /wheel/odom  ⽅向是否正确（真机最常⻅问题）。
> 2. 再查  /imu/data  零偏是否过⼤。
> 3. 再查  covariance  是否全 0（导致 EKF 权重错误）。
> 4. 再查  robot_localization  的  world_frame  是否为  odom （不是  map ）。
> 5. 最后才调 EKF 参数。

**先修底层传感器，不要急着调 Nav2**。

# 7. Nav2 参数（真机微调）

## 7.1 复⽤仿真参数结构

直接把第⼀周验证过的  nav2_params.yaml  复制到真机 Package 中，然后做以下修改。这⾥的“复⽤”指
复⽤参数结构和保守初始值，不代表仿真参数可以作为真机最终值。

```yaml
# 关键修改：use_sim_time 全部改为 false
amcl:
  ros__parameters:
    use_sim_time: false

bt_navigator:
  ros__parameters:
    use_sim_time: false

controller_server:
  ros__parameters:
    use_sim_time: false

planner_server:
  ros__parameters:
    use_sim_time: false

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
```
修改后做⼀次全局检查，确保没有残留的  use_sim_time: true ：
```sh
grep -n "use_sim_time" $HOME/robot_nav_ws/src/robot_nav_bringup/config/nav2_params.yaml
```

## 7.2 真机需要微调的地⽅

仿真值               | 真机调整⽅向   | 原因
--------------------|--------------|--------
max_vel_x: 0.15     | 真机通常可以适度提⾼，但第⼀版建议先保持0.15 | 真机动⼒学和仿真不同，先保守
max_vel_theta: 0.35 | 如果机器⼈转向迟缓，可适度提⾼             | 真机摩擦和仿真不同 
acc_lim_x: 0.25     | 如果起停太猛，降低；如果太⾁，提⾼          | 真机电机响应不同
robot_radius: 0.30  | 改为真实机器⼈外接圆半径                  | 仿真尺⼨不⼀定等于真实
inflation_radius: 0.70 |  根据真机场地调整                     | ⾛廊宽窄不同
xy_goal_tolerance:0.30 |  如果机器⼈在⽬标点附近抖动，调⼤        | 真机控制精度不如仿真

## 7.3  footprint  的切换

如果机器⼈是矩形、⽅形、前后不对称，仿真⾥⽤了  robot_radius  的，真机阶段应改为
footprint ：
```
# 删除 robot_radius: 0.30
footprint: "[[0.30, 0.20], [0.30, -0.20], [-0.30, -0.20], [-0.30, 0.20]]"
```
注意：在  local_costmap  和  global_costmap  中都要改，且不要保留  robot_radius 。

# 8. SLAM 建图（真机）

## 8.1 启动 SLAM Toolbox
```sh
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```
注意： use_sim_time  改为  false 。

## 8.2 启动真机链路

顺序：
> 1. 底盘驱动
> 2. 雷达驱动
> 3. IMU 驱动
> 4.  robot_state_publisher
> 5.  robot_localization
> 6.  slam_toolbox
> 7.  rviz2

建议写⼀个统⼀的  bringup.launch.py  来管理这个顺序，不要⼿动逐个开终端。

## 8.3 ⽤键盘或遥控移动机器⼈
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

或⽤⼿柄遥控。在真机中：
* 低速直⾏。
* 低速转弯。
* 尽量绕成闭环。
* 不要⻓时间原地⾼速旋转。
* 不要第⼀次就进强反光、玻璃多、纹理极少的场地。
* 确保急停随时可⽤。

## 8.4 建图阶段的现象

正常：
* 激光扇形跟随机器⼈移动。
* 地图边⾛边⻓出来。
* 绕回起点时，地图不会明显裂开成两层。
* 机器⼈停下时，位姿不持续漂移。

异常：
* 激光扇形在机器⼈后⾯（ base_link  前向定义错误）。
* 机器⼈向前⾛，地图反着移动（ odom  ⽅向错误）。
* 原地慢转时地图整体撕裂（EKF 融合或  odom  质量不够）。
* 机器⼈静⽌时位姿持续漂移（EKF 参数或 IMU 零偏问题）。

# 9. 保存地图
```sh
mkdir -p $HOME/robot_nav_ws/maps

ros2 run nav2_map_server map_saver_cli -f $HOME/robot_nav_ws/maps/demo_map
```
⽣成  demo_map.yaml  和  demo_map.pgm 。

# 10. 切换到导航模式（真机）

## 10.1 切换逻辑

* 停⽌  slam_toolbox
* 保留底盘、传感器、TF、 robot_localization
* 启动  map_server
* 启动  amcl
* 启动 Nav2 其他服务器

## 10.2 为什么不让  slam_toolbox  和  amcl  同时运⾏

两者都可能发布  map -> odom 。如果同时发布：
* RViz 位姿会跳。
* Nav2 规划会飘。
* 你会以为是控制器坏了，其实是全局定位源冲突。

## 10.3 Lifecycle Node 检查

```sh
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```
bringup_launch.py  会⾃动激活，但⼿动排查时可⽤这些命令。

## 10.4 Behavior Tree 提醒

如果机器⼈原地旋转，先判断这是 Nav2 的 recovery ⾏为（清理 Costmap / 旋转 / 后退）还是故障。真
机中 recovery ⾏为更常⻅，因为场地环境⽐仿真复杂。

## 10.5 导航模式启动
```sh
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/robot_nav_ws/maps/demo_map.yaml \
  use_sim_time:=false \
  params_file:=$HOME/robot_nav_ws/src/robot_nav_bringup/config/nav2_params.yaml
```

启动后检查真机 Parameter 是否真的⽣效：
```sh
ros2 param dump /controller_server | grep -E "max_vel_x|max_vel_theta"
ros2 param dump /local_costmap/local_costmap | grep -E "robot_radius|inflation_radius"
```

## 10.6 RViz 操作
> 1. ⽤  2D Pose Estimate  给出初始位姿。
> 2. 等待  amcl  收敛（通常⼏秒到⼗⼏秒）。
> 3. ⽤  Nav2 Goal  发送⽬标点。

如果初始位姿就错了，后⾯再怎么调控制参数也不会顺。


## 10.7 导航阶段的正常现象

* 设完初始位姿后，机器⼈位姿有⼀个短暂收敛过程。
* 发⽬标点后，先出现全局路径，再出现局部控制⾏为。
* 遇到障碍物或局部路径受阻时，机器⼈可能会短暂停顿、转向或尝试恢复。

真正需要警惕的是：
* 全局路径完全出不来。
* /cmd_vel  ⻓时间没有任何输出。
* 机器⼈持续⼤⻆度原地乱转且没有恢复趋势。
* 机器⼈朝⽬标反⽅向稳定运动。

# 11. 第⼀个完整⽤例（真机）

## 11.1 ⽤例名称

NAV-CASE-001-REAL：真机室内建图与定点导航

## 11.2 前置条件

* 急停已验证有效。
* 轮⼦悬空或空载低速测试已通过。
* /scan  频率稳定， frame_id  正确。
* /wheel/odom  前进⽅向和 yaw ⽅向正确。
* /imu/data  yaw ⽅向正确，静⽌时⻆速度⽆明显漂移。
* TF 树完整， base_link -> laser 、 base_link -> imu_link  可查。
* 机器⼈可⼿动低速移动，且  /cmd_vel  能被底盘正确执⾏。
* 仿真参数已复制到真机 Package，并已把  use_sim_time  改为  false 。
* Nav2 速度上限处于保守值，例如  max_vel_x <= 0.15 。

## 11.3 步骤
> 1. 启动真机底盘、雷达、IMU 驱动。
> 2. 启动  robot_state_publisher 。
> 3. 启动  robot_localization 。
> 4. 启动  slam_toolbox 。
> 5. ⼿动低速移动机器⼈完成建图。
> 6. 保存地图。
> 7. 停⽌  slam_toolbox 。
> 8. 启动  map_server + amcl + Nav2 。
> 9. 在 RViz 中设置初始位姿。
> 0. 发送 2-5 ⽶导航⽬标。
> 1. 观察路径规划与执⾏。

## 11.4 通过标准

* 地图保存成功。
* amcl  和 Nav2 Node 进⼊  active  状态。
* 能⽣成全局路径。
* 机器⼈低速避障到达⽬标。
* 没有碰撞。
* 没有明显失控。
* ⽬标误差在⾸版容差范围内。

# 12. 调试与诊断（真机）

## 12.1 最⼩调试闭环

> 1.  ros2 topic list
> 2.  ros2 topic hz /scan
> 3.  ros2 topic echo /scan --once
> 4.  ros2 topic echo /wheel/odom --once
> 5.  ros2 topic echo /imu/data --once
> 6.  ros2 run tf2_tools view_frames
> 7. 启动  slam_toolbox
> 8. 打开 RViz 看地图和激光
> 9. 保存地图
> 0. 启动导航模式
> 1. 设置初始位姿
> 2. 发第⼀个⽬标

## 12.2 卡住时的回退规则

遇到问题时按这个顺序回退：
> 1. 先查 Topic 是否存在。
> 2. 再查 Topic 频率是否稳定。
> 3. 再查消息⾥的  frame_id  和时间戳。
> 4. 再查 TF 是否连通。
> 5. 再查传感器数据⽅向（前/后/左/右是否颠倒）。
> 6. 最后才查 Nav2 参数。

很多“Nav2 不好⽤”的问题，本质上是雷达、⾥程计、IMU、TF 中有⼀个没站稳。底层没稳时，继续往上
调 Nav2 参数只会让问题更难定位。

## 12.3 TF 诊断
```sh
ros2 run tf2_tools view_frames
```
⽣成 TF 树 PDF，看有没有断链、Frame 名字是否拼错。

## 12.4 Lifecycle 状态
```sh
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

## 12.5 查看实际参数
```sh
ros2 param dump /controller_server
```
检查你以为改了的参数，是否真的⽣效。

## 12.6 清理局部 Costmap
```sh
ros2 service call /local_costmap/clear_entirely_local_costmap \
  nav2_msgs/srv/ClearEntireCostmap "{}"
```
不同版本的服务名可能略有差异，遇到不⼀致时先：
```sh
ros2 service list | grep costmap
```
## 12.7 观察 Behavior Tree ⽇志
```sh
ros2 topic echo /behavior_tree_log
```
如果没有这个 Topic，先回到 RViz 和 Node ⽇志排查。

## 13. 常⻅故障（真机）

## 13.1 没有地图
 
* slam_toolbox  是否启动
* /map  是否有数据
* RViz Fixed Frame 是否为  map
* use_sim_time  是否⼀致（真机应为  false ）
* 时间戳是否⼀致（检查雷达和 IMU 的时钟是否同步）

## 13.2 Nav2 不动

* /cmd_vel  是否有输出
* 底盘是否真正订阅了  /cmd_vel
* 急停是否释放
* Lifecycle Node 是否  active
* Costmap 是否把机器⼈包在障碍⾥（检查雷达最近点是否被本体遮挡）

## 13.3 原地转圈

先判断这是 recovery 还是故障。如果怀疑故障：

* 初始位姿错
* TF 延迟或跳变
* IMU ⽅向错（真机中最常⻅）
* 轮速⽅向错（真机中第⼆常⻅）
* yaw_goal_tolerance  太⼩
* ⽬标点设置不合理（⽐如就在机器⼈正后⽅）

## 13.4 贴墙或离障碍物太近

* robot_radius 或 footprint（真机尺⼨⼀定要准）
* inflation_radius
* cost_scaling_factor
* 雷达是否被线缆、壳体遮挡（真机常⻅问题）

## 13.5 路径规划出来了，但机器⼈不跟

* /cmd_vel  是否真发了
* 底盘速度上限是否被底层控制器夹死（例如底盘固件限制了最⼤速度）
* odom -> base_link  是否平滑（检查 EKF 输出是否跳变）
* 局部 Costmap 是否被旧障碍物污染（尝试清理 Costmap）

## 13.6 地图撕裂或漂移

* 机器⼈原地旋转时是否过快（真机电机可能⽐仿真更猛）
* 雷达频率是否过低（低于 10 Hz 时 SLAM 容易跟丢）
* 场地是否过于空旷（SLAM 缺乏特征点）
* EKF 融合是否发散（检查  odometry/filtered  是否平滑）


# 14. 室外扩展⽅向

第⼀阶段不做室外主链路，但建议⼀开始就给接⼝留位置：
* GPS / RTK
* IMU（已接⼊）
* 轮速⾥程计（已接⼊）
* 激光避障（已接⼊）
* geofence 或 keepout 区域

室外模式的第⼀步不是“上⻓距离巡航”，⽽是：
* 低速
* 开阔区域
* 短距离 waypoint
* 先确认 GPS / RTK 与 odom 融合稳定

# 15. 真机阶段交付物

完成第⼆周时，⾄少交付：
* ⼀个可构建的 ROS2 workspace
* ⼀份真实尺⼨的机器⼈ URDF / xacro
* ⼀份  robot_localization  参数⽂件（经过真机验证）
* ⼀份 Nav2 参数⽂件（经过真机验证）
* ⼀份真机 Launch ⽂件（ bringup.launch.py ）
* ⼀张真实场地地图（ .yaml + .pgm ）
* ⼀次成功的真机定点导航（附 RViz 截图或 rosbag）
* ⼀份 rosbag（记录建图和导航过程）
* ⼀份问题清单（记录仿真和真机的差异、待优化项）

# 16. 真机⼀天路线

如果这是⼯程师第⼀次真正把系统接到真机上，建议按⼀天节奏推进。

## 16.1 上午：把硬件和底层数据跑通

⽬标：Topic 可⻅、频率稳定、⽅向正确、TF 完整。

动作：
> 1. 启动底盘、雷达、IMU 驱动。
> 2. ⽤  ros2 topic list 、 ros2 topic hz 、 ros2 topic echo --once  做传感器检查。
> 3. 启动  robot_state_publisher 。
> 4. 检查  view_frames 。

通关标准：
* Topic 存在、频率稳定。
* frame_id  正确。
* 前进⽅向正确、转向⽅向正确。
* TF ⽆断链。

## 16.2 下午前半段：把 EKF 和建图跑通

⽬标：EKF 输出平滑， slam_toolbox  能开始建图。

动作：
> 1. 复制仿真 EKF 参数，启动  robot_localization 。
> 2. 检查  /odometry/filtered  是否平滑、⽆跳变。
> 3. 启动  slam_toolbox 。
> 4. ⼿动低速移动机器⼈，打开 RViz 观察地图展开。

通关标准：
* 地图随着机器⼈运动稳定⽣成。
* 没有明显地图撕裂。
* 机器⼈静⽌时位姿不过度漂移。
* EKF 输出没有异常跳变。

## 16.3 下午后半段：把保存地图和导航跑通

⽬标：保存静态地图，切换到导航模式，发⼀个⽬标点并完成导航。

动作：
1. ⽤  map_saver_cli  保存地图。
2. 停⽌  slam_toolbox 。
3. 复制仿真 Nav2 参数（修改  use_sim_time  为  false ），启动  map_server + amcl + Nav2 。
4. 在 RViz 中设置初始位姿。
5. 发 2-5 ⽶⽬标点。

通关标准：
* 全局路径出现。
* /cmd_vel  有输出。
* 机器⼈能低速到达⽬标附近。
* 没有碰撞或明显失控。

如果⼀天结束还没到导航，不算失败。对第⼀次接触真机的⼯程师来说，能把底层 Topic、TF 和 SLAM 稳
定跑起来，已经是⾮常好的进度。导航参数可以第⼆天再细调。

# 17. 延伸阅读与官⽅⽂档

## 17.1 推荐阅读路径

> 1. 先完成上部仿真指南，建⽴系统⻣架。
> 2. 真机接⼊时，看 Nav2 First-Time Robot Setup Guide 的真机相关章节。
> 3. 调 footprint 和 inflation 时，看 Nav2 Costmap 2D ⽂档。
> 4. 遇到 Lifecycle 或 Behavior Tree 现象时，回头看 Nav2 Concepts。

## 17.2 官⽅⽂档精确参考

### Nav2 First-Time Robot Setup Guide

https://docs.nav2.org/setup_guides/index.html

重点看：真机 TF、URDF、⾥程计、EKF、footprint、Nav2 Plugin 接⼊。

### Nav2 SLAM 教程

https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html

### Nav2 Costmap 2D

https://docs.nav2.org/configuration/packages/configuring-costmaps.html

### Nav2 Concepts

https://docs.nav2.org/concepts/index.html

重点看：Lifecycle Nodes、Behavior Trees、Navigation Servers、Robot Footprints、State

Estimation。

### 室外 GPS 导航

https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html

适合第⼀阶段室内链路跑通后，进⼊第⼆阶段室外扩展。

# 附录：仿真 → 真机迁移对照表

项⽬          | 仿真（上部）   | 真机（下部）
-------------|--------------|---------------
use_sim_time | true         | false
底盘驱动      | gazebo_ros_diff_drive | 真实底盘驱动 Node
雷达驱动      | Gazebo 激光插件        | 真实雷达驱动 Node
IMU 驱动     | Gazebo IMU 插件        | 真实 IMU 驱动 Node
场地         | Gazebo  .world        | 真实室内场地
URDF        | 示范尺⼨              | 真实测量尺⼨
EKF 参数     | 仿真默认值            | 微调协⽅差和权重
Nav2 速度参数 | 保守值（0.15 m/s）    | 根据真机动⼒学微调
数据检查      | 确认数据流动          | 确认⽅向、频率、协⽅差、噪声
故障类型      | 多数是配置或参数结构问题 | 硬件⽅向错、遮挡、时钟不同步、协⽅差不合理
交付物        | 仿真参数包  | 真机验证的完整系统


# 最后的话

如果你已经按照上部完成了仿真闭环，那么第⼆周的核⼼任务其实只有三件事：
> 1. 把硬件驱动接进来，让  /scan 、 /wheel/odom 、 /imu/data  和仿真时⼀样⼲净。
> 2. 把  use_sim_time  改成  false 。
> 3. 根据真机表现微调 EKF 和 Nav2 参数。

仿真阶段已经帮你排除了⼤部分“概念错误”和“参数结构错误”。真机阶段遇到的，常常是“硬件细节问题”
⸺⽅向反了、频率不稳、雷达被遮挡、协⽅差没填、时间戳不连续。这些问题排查起来有⽅法，不需要
重新学⼀遍 Nav2 架构，但必须逐项验证。

先把仿真闭环跑通，第⼆周上真机时你会感谢第⼀周的积累。

