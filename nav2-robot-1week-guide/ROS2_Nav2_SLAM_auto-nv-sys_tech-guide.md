# ROS2 / Nav2 + SLAM 机器人自主导航系统技术指导
<!-- 
https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_on_real_turtlebot3/navigation2_on_real_turtlebot3/ 
-->

面向对象：具备 Linux、Python/C++、基础 ROS 概念的软件工程师。  
目标：指导工程师在仿真或真实四轮机器人平台上，搭建一套基于 ROS2、Nav2、SLAM 的自主导航系统，并跑通第一个“建图 -> 保存地图 -> 定位 -> 发送导航目标”的闭环用例。  
项目参考目录：`/Users/dj/Documents/test_coding/nav_harness_robot`

> 本文档是工程启动手册，不是安全认证文件。真实机器人测试前必须具备急停、人工接管、低速限制、障碍物安全距离和测试人员监护。

---

## 1. 系统目标

本系统面向室内外巡检机器人样机，核心目标如下：

- 使用 ROS2 作为机器人软件中间件。
- 使用 Nav2 作为自主导航框架。
- 使用 SLAM Toolbox 完成室内建图。
- 使用激光雷达、轮速里程计、IMU 作为基础导航传感器。
- 室外开放区域可扩展 GPS / RTK-GNSS 融合定位。
- 首个用例跑通：机器人在未知室内环境中建图，保存地图，重新加载地图并导航到目标点。

推荐首个阶段只做室内低速验证。室外 GPS / RTK、3D 激光、视觉巡检业务可作为第二阶段扩展。

---

## 2. 推荐软硬件基线

### 2.1 计算平台

推荐两种开发方式：

| 场景 | 推荐平台 | 说明 |
| --- | --- | --- |
| 开发调试 | x86 Ubuntu 工作站 | 适合仿真、RViz、日志分析 |
| 机器人本体 | NVIDIA Jetson Orin Nano 或同级边缘计算单元 | 适合部署导航、传感器驱动、业务节点 |

如果当前项目使用 RK3588，也可以作为导航计算平台，但应提前验证 3D SLAM、视觉算法、业务服务并发时的 CPU / GPU / NPU 负载和散热。

### 2.2 机器人本体

建议基础配置：

- 四轮差速或滑移转向底盘。
- 电机控制器可接收 `/cmd_vel`。
- 底盘可发布 `/wheel/odom`。
- 2D 激光雷达，发布 `/scan`。
- IMU，发布 `/imu/data`。
- 可选 GPS / RTK，发布 `/fix`。
- 电池、电源、急停、遥控接管。

### 2.3 ROS 话题约定

| 功能 | 推荐话题 | 消息类型 |
| --- | --- | --- |
| 速度指令   | `/cmd_vel`    | `geometry_msgs/Twist` |
| 激光雷达   | `/scan`       | `sensor_msgs/LaserScan` |
| 轮速里程计 | `/wheel/odom` | `nav_msgs/Odometry` |
| IMU      | `/imu/data`   | `sensor_msgs/Imu` |
| 融合里程计 | `/odometry/filtered` | `nav_msgs/Odometry` |
| 地图      | `/map`               | `nav_msgs/OccupancyGrid` |
| GPS      | `/fix`               | `sensor_msgs/NavSatFix` |

### 2.4 TF 坐标系约定

| 坐标系 | 含义 |
| --- | --- |
| `map` | 全局地图坐标系 |
| `odom` | 局部连续里程计坐标系 |
| `base_link` | 机器人本体坐标系 |
| `laser` | 激光雷达坐标系 |
| `imu_link` | IMU 坐标系 |
| `gps_link` | GPS 天线坐标系 |

Nav2 对 TF 非常敏感。第一天调试中，很多问题不是算法问题，而是 TF 名称、方向、时间戳错误。

---

## 3. 软件栈选择

### 3.1 推荐版本

建议优先选择：
- Ubuntu 22.04 + ROS2 Humble，稳定、生态成熟。
- Ubuntu 24.04 + ROS2 Jazzy，较新，适合新项目长期维护。

如果目标是 Jetson Orin Nano，先确认 JetPack 对 Ubuntu 版本的支持，再选择对应 ROS2 发行版。不要先写死 ROS2 版本，再回头发现 JetPack 不匹配。

### 3.2 核心软件包

必须安装：
- `navigation2`
- `nav2_bringup`
- `slam_toolbox`
- `robot_localization`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`
- `colcon`

可选安装：

- Gazebo / Ignition / Gazebo Harmonic 仿真环境。
- 具体雷达、IMU、GPS、底盘驱动。
- `teleop_twist_keyboard`，用于手动遥控建图。

---

## 4. 项目目录说明

本文档假设使用已有项目：

```bash
cd /Users/dj/Documents/test_coding/nav_harness_robot
```

关键目录：

```sh
nav_harness_robot/
  src/
    nav_harness_bringup/
      launch/
        nav_system.launch.py
        sim.launch.py
        gps_nav.launch.py
      config/
        nav2_params.yaml
        outdoor_nav2_params.yaml
        robot_localization.yaml
        sensors.yaml
      urdf/
        four_wheel_robot.urdf.xacro
      maps/
    nav_harness_tools/
  harness/
  issues/
  docs/
```

第一阶段主要修改：

- `src/nav_harness_bringup/config/nav2_params.yaml`
- `src/nav_harness_bringup/config/robot_localization.yaml`
- `src/nav_harness_bringup/urdf/four_wheel_robot.urdf.xacro`
- `src/nav_harness_bringup/launch/nav_system.launch.py`

---

## 5. 环境搭建

以下以 Ubuntu + ROS2 为例。命令中的 `$ROS_DISTRO` 可以是 `humble` 或 `jazzy`。

### 5.1 安装 ROS2

请按 ROS2 官方文档安装对应发行版。安装完成后确认：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 --version
```

建议把 source 写入 shell 启动文件：

```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
```

### 5.2 安装导航依赖

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep
```

可选：

```bash
sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
```

### 5.3 初始化 rosdep

如果机器第一次使用 ROS：

```bash
sudo rosdep init
rosdep update
```

如果 `rosdep init` 提示已经初始化，可以跳过。

---

## 6. 构建项目

进入项目根目录：

```bash
cd /Users/dj/Documents/test_coding/nav_harness_robot
```

安装依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

构建：

```bash
colcon build --symlink-install
```

加载工作空间：

```bash
source install/setup.bash
```

检查包是否可见：

```bash
ros2 pkg list | grep nav_harness
```

预期看到：

```text
nav_harness_bringup
nav_harness_tools
```

---

## 7. 第一个用例概览

第一个用例分为 5 步：

> 1. 启动机器人底盘、雷达、IMU、TF。
> 2. 使用 SLAM Toolbox 建图。
> 3. 使用键盘或遥控器低速移动机器人。
> 4. 保存地图。
> 5. 使用 Nav2 加载地图并发送导航目标。

如果暂时没有真实机器人，可以先用仿真替代第 1 步。真实机器人与仿真的核心要求一样：话题和 TF 必须一致。

---

## 8. 传感器和底盘接口检查

在运行 Nav2 前，先检查底层接口。

### 8.1 检查话题

```bash
ros2 topic list
```

至少应看到：

```text
/scan
/wheel/odom
/imu/data
/cmd_vel
/tf
/tf_static
```

检查雷达频率：

```bash
ros2 topic hz /scan
```

检查里程计频率：

```bash
ros2 topic hz /wheel/odom
```

检查 IMU 频率：

```bash
ros2 topic hz /imu/data
```

### 8.2 检查消息内容

```bash
ros2 topic echo /scan --once
ros2 topic echo /wheel/odom --once
ros2 topic echo /imu/data --once
```

重点看：

- `header.frame_id` 是否正确。
- `stamp` 是否持续更新。
- 雷达 range 是否不是全 0 或全 NaN。
- 轮速里程计方向是否与机器人实际运动一致。

### 8.3 检查 TF

```bash
ros2 run tf2_tools view_frames
```

生成 `frames.pdf` 后检查坐标树是否包含：

```text
map -> odom -> base_link -> laser
                         -> imu_link
```

建图阶段没有 `map -> odom` 也正常，因为它通常由 SLAM 提供。导航阶段 `map -> odom` 必须存在。

---

## 9. URDF 和传感器安装参数

机器人模型在：

```text
src/nav_harness_bringup/urdf/four_wheel_robot.urdf.xacro
```

需要按实际机械结构修改：

- 机器人长度、宽度、高度。
- 轮子半径、轮距。
- 雷达相对 `base_link` 的位置。
- IMU 相对 `base_link` 的位置。
- GPS 天线相对 `base_link` 的位置。

例如雷达安装：

```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser"/>
  <origin xyz="0.26 0 0.27" rpy="0 0 0"/>
</joint>
```

这里的 `xyz` 必须用尺子实测。单位是米。  
如果雷达实际朝向与 `base_link` 不一致，要修改 `rpy`。

---

## 10. 使用 SLAM 建图

### 10.1 启动底层驱动

真实机器人上先启动：
- 底盘驱动。
- 雷达驱动。
- IMU 驱动。
- `robot_state_publisher`。
- 必要的静态 TF。

项目中 `nav_system.launch.py` 已包含部分 bringup，但具体传感器驱动需要根据硬件型号接入。

### 10.2 启动 SLAM Toolbox

可直接使用官方在线异步建图模式：

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

如果是仿真：

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

### 10.3 启动 RViz

```bash
rviz2
```

在 RViz 中添加：
- `Map`
- `LaserScan`
- `TF`
- `Odometry`

Fixed Frame 设置为：
```text
map
```

### 10.4 手动移动机器人

建议第一次使用极低速：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

控制原则：

- 慢速直行。
- 慢速转弯。
- 不要快速原地旋转。
- 尽量走闭环路径，帮助 SLAM 回环优化。
- 避免在玻璃、纯白墙、强反光区域长时间建图。

---

## 11. 保存地图

建图完成后保存地图：

```bash
mkdir -p src/nav_harness_bringup/maps
ros2 run nav2_map_server map_saver_cli \
  -f src/nav_harness_bringup/maps/inspection_area
```

预期生成：

```text
src/nav_harness_bringup/maps/inspection_area.yaml
src/nav_harness_bringup/maps/inspection_area.pgm
```

检查 YAML：

```bash
cat src/nav_harness_bringup/maps/inspection_area.yaml
```

典型内容：

```yaml
image: inspection_area.pgm
mode: trinary
resolution: 0.05
origin: [...]
occupied_thresh: 0.65
free_thresh: 0.25
```

---

## 12. 启动 Nav2 定位导航

停止 SLAM 建图节点后，启动 Nav2：

```bash
ros2 launch nav_harness_bringup nav_system.launch.py \
  mode:=indoor \
  map:=src/nav_harness_bringup/maps/inspection_area.yaml \
  use_sim_time:=false
```

如果是仿真：

```bash
ros2 launch nav_harness_bringup nav_system.launch.py \
  mode:=indoor \
  map:=src/nav_harness_bringup/maps/inspection_area.yaml \
  use_sim_time:=true
```

打开 RViz：

```bash
rviz2
```

在 RViz 中设置：

- Fixed Frame: `map`
- 添加 `Map`
- 添加 `LaserScan`
- 添加 `TF`
- 添加 `Path`
- 添加 `RobotModel`

然后执行：

1. 使用 `2D Pose Estimate` 设置初始位姿。
2. 使用 `Nav2 Goal` 或 `2D Goal Pose` 发送目标点。
3. 观察全局路径、局部轨迹、速度指令和机器人运动。

---

## 13. Nav2 参数文件说明

主要参数文件：

```text
src/nav_harness_bringup/config/nav2_params.yaml
```

这个文件决定机器人导航行为。

### 13.1 控制器参数

位置：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.2
      max_vel_theta: 0.5
      acc_lim_x: 0.4
      acc_lim_theta: 0.8
```

含义：

| 参数 | 含义 | 初始建议 |
| --- | --- | --- |
| `max_vel_x` | 最大前进速度 | 0.1-0.2 m/s |
| `max_vel_theta` | 最大角速度 | 0.3-0.5 rad/s |
| `acc_lim_x` | 前进加速度限制 | 0.2-0.4 m/s² |
| `acc_lim_theta` | 角加速度限制 | 0.4-0.8 rad/s² |

真实机器人第一次测试时，不要追求速度。先追求稳定、安全、可复现。

### 13.2 目标容差

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.25
  yaw_goal_tolerance: 0.25
```

如果机器人到目标附近反复微调、抖动，可以适当放宽：

```yaml
xy_goal_tolerance: 0.35
yaw_goal_tolerance: 0.5
```

### 13.3 障碍物安全距离

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.28
      inflation_layer:
        inflation_radius: 0.55
        cost_scaling_factor: 3.0
```

关键原则：

- `robot_radius` 必须大于机器人真实半宽。
- 如果机器人是长方形，应改用 `footprint`。
- `inflation_radius` 越大，机器人离障碍物越远。
- 狭窄通道中 inflation 太大可能导致无法规划。

### 13.4 激光雷达范围

```yaml
scan:
  topic: /scan
  raytrace_max_range: 5.0
  obstacle_max_range: 4.0
```

建议：

- `obstacle_max_range` 不要超过雷达稳定有效距离。
- `raytrace_max_range` 通常略大于 `obstacle_max_range`。
- 雷达安装过低会看不到桌面、玻璃、悬空障碍物。

---

## 14. 轮速里程计和 IMU 融合

融合配置在：

```text
src/nav_harness_bringup/config/robot_localization.yaml
```

当前使用 `robot_localization` 的 EKF：

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    odom0: /wheel/odom
    imu0: /imu/data
```

调试重点：

- 机器人向前走，`/wheel/odom` 的 x 应增加。
- 机器人左转，yaw 应按 ROS 坐标约定变化。
- IMU 安装方向必须正确。
- IMU yaw 漂移会影响局部控制稳定性。

如果融合后机器人在 RViz 中飘移明显，先不要调 Nav2，先修 odom / IMU / TF。

---

## 15. 第一个验收用例

### 15.1 用例名称

`NAV-CASE-001：室内建图与定点导航`

### 15.2 前置条件

- 机器人能低速手动移动。
- `/scan` 正常。
- `/wheel/odom` 正常。
- `/imu/data` 正常。
- TF 树没有断裂。
- 急停和人工接管可用。

### 15.3 步骤

1. 启动底盘、雷达、IMU。
2. 启动 SLAM Toolbox。
3. 用遥控或键盘控制机器人绕场地移动。
4. 保存地图。
5. 停止 SLAM。
6. 启动 Nav2 并加载保存的地图。
7. 在 RViz 中设置初始位姿。
8. 发送一个距离 2-5 米的导航目标。
9. 观察机器人是否规划路径并到达目标。

### 15.4 通过标准

- 地图能正常保存和重新加载。
- Nav2 所有 lifecycle 节点进入 active 状态。
- RViz 中机器人位姿与真实位置基本一致。
- 能生成全局路径。
- 机器人能以低速避障并到达目标。
- 目标误差小于 0.3 米。
- 无碰撞、无突然加速、无持续原地振荡。

---

## 16. 日志记录

每次测试都应记录 rosbag：

```bash
scripts/record_test_bag.sh
```

该脚本记录：

```text
/tf
/tf_static
/scan
/fix
/imu/data
/wheel/odom
/odometry/filtered
/cmd_vel
/plan
```

如果脚本中某些话题暂时不存在，可以先手动记录核心话题：

```bash
ros2 bag record /tf /tf_static /scan /wheel/odom /imu/data /cmd_vel
```

测试后记录：

- 测试日期。
- 地点。
- 地图文件。
- 参数文件版本。
- 是否通过。
- 发现的问题。

---

## 17. 常见问题排查

### 17.1 RViz 中没有地图

检查：

```bash
ros2 topic echo /map --once
```

可能原因：

- SLAM 没启动。
- Map Server 没启动。
- Fixed Frame 不是 `map`。
- `use_sim_time` 设置不一致。

### 17.2 Nav2 一直 inactive

检查 lifecycle：

```bash
ros2 lifecycle nodes
```

查看日志中是否有：

- 参数文件错误。
- TF timeout。
- map 文件路径错误。
- 插件名称错误。

### 17.3 机器人不动

检查：

```bash
ros2 topic echo /cmd_vel
```

如果 `/cmd_vel` 有速度但机器人不动：

- 底盘驱动没有订阅 `/cmd_vel`。
- 话题名不一致。
- 底盘急停未释放。
- 控制器限速为 0。

如果 `/cmd_vel` 没有速度：

- Nav2 未 active。
- 没有收到目标。
- 局部规划失败。
- costmap 中机器人被障碍物包围。

### 17.4 机器人离障碍物太近

优先调整：

```yaml
robot_radius
inflation_radius
cost_scaling_factor
```

同时检查机器人真实外形是否应该用 `footprint`。

### 17.5 机器人原地转圈或路径抖动

可能原因：

- 初始位姿错误。
- TF 延迟或跳变。
- IMU yaw 方向错误。
- 轮速里程计比例错误。
- 目标 yaw 容差太小。
- 控制器速度 / 加速度过激。

先降低速度：

```yaml
max_vel_x: 0.1
max_vel_theta: 0.3
acc_lim_x: 0.2
acc_lim_theta: 0.4
```

---

## 18. 室外 GPS / RTK 扩展方向

室外开阔区域建议加入：

- RTK-GNSS。
- IMU。
- 轮速里程计。
- 2D / 3D 激光避障。
- 地理围栏。

项目中已有入口：

```text
src/nav_harness_bringup/launch/gps_nav.launch.py
src/nav_harness_bringup/config/outdoor_nav2_params.yaml
```

室外模式建议：

- 放宽目标容差，例如 `0.75-1.5 m`。
- 保持低速。
- 设置禁行区或 geofence。
- 对 GPS 协方差做门限检查。
- 避免在高楼、树荫、金属结构附近首次测试。

室外第一阶段不要直接做长距离自主巡检，应先做 5-10 米低速 waypoint 测试。

---

## 19. 工程协作和问题管理

本项目带有本地 issue board：

```text
issues/issue_board.html
```

建议每个问题至少写：

- 问题标题。
- 环境：仿真 / bench / 室内 / 室外。
- 优先级：P0 / P1 / P2 / P3。
- 复现步骤。
- 证据：rosbag、截图、日志。
- 验收条件。

优先级建议：

| 优先级 | 含义 |
| --- | --- |
| P0 | 安全风险、碰撞、失控、急停失效 |
| P1 | 阻塞导航主流程 |
| P2 | 影响稳定性或体验 |
| P3 | 文档、清理、优化 |

---

## 20. 第一周建议工作计划

### Day 1：环境和构建

- 安装 ROS2、Nav2、SLAM Toolbox。
- 构建 `nav_harness_robot`。
- 确认包可见。

### Day 2：接口检查

- 接入雷达。
- 接入底盘 odom。
- 接入 IMU。
- 检查 `/scan`、`/wheel/odom`、`/imu/data`。

### Day 3：TF 和 URDF

- 修改机器人尺寸。
- 修改雷达、IMU 安装位置。
- 生成 TF 树。
- 在 RViz 中检查 RobotModel。

### Day 4：SLAM 建图

- 启动 SLAM Toolbox。
- 手动低速建图。
- 保存地图。

### Day 5：Nav2 导航

- 加载地图。
- 设置初始位姿。
- 发送第一个目标点。
- 记录 rosbag 和问题。

---

## 21. 最小成功标准

工程师完成本文档后，应至少交付：

- 可构建的 ROS2 工作空间。
- 一张可加载的室内地图。
- 一份 Nav2 参数文件。
- 一份 TF 树截图或 `frames.pdf`。
- 一个 rosbag 测试记录。
- 一条成功的短距离导航用例。
- issue board 中记录的待解决问题。

---

## 22. 官方参考资料

- ROS2 官方文档：https://docs.ros.org/
- ROS2 发行版说明：https://docs.ros.org/en/rolling/Releases.html
- Nav2 官方文档：https://docs.nav2.org/
- Nav2 First-Time Robot Setup Guide：https://docs.nav2.org/setup_guides/index.html
- Nav2 Navigating while Mapping：https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html
- Nav2 Tuning Guide：https://docs.nav2.org/tuning/index.html
- robot_localization 文档：http://docs.ros.org/en/noetic/api/robot_localization/html/

---

## 23. 附录：推荐初始参数

真实机器人首次测试建议：

```yaml
FollowPath:
  max_vel_x: 0.15
  max_speed_xy: 0.15
  max_vel_theta: 0.35
  acc_lim_x: 0.25
  acc_lim_theta: 0.5
  decel_lim_x: -0.25
  decel_lim_theta: -0.5
```

安全距离建议：

```yaml
robot_radius: 0.30
inflation_layer:
  inflation_radius: 0.70
  cost_scaling_factor: 3.0
```

目标容差建议：

```yaml
xy_goal_tolerance: 0.30
yaw_goal_tolerance: 0.50
```

如果机器人稳定后，再逐步提高速度。每次只改一组参数，并记录测试结果。

