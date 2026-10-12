# 6. 接口约定 落地实现（ROS2 Humble，配套前面的xacro+Gazebo）

> 
> 本节目标：**保证话题名、frame、TF发布者完全和上面表格对齐**，分4大块：
> 
> 
> 1. 修改Xacro/Gazebo插件，强制输出约定topic
> 2. robot_state_publisher 启动（发布URDF内静态TF：base_link→laser / base_link→imu_link）
> 3. robot_localization 配置（ekf_node，输入/wheel/odom + /imu/data，输出`/odometry/filtered`、odom→base_link）
> 4. SLAM Toolbox / AMCL 分工（map→odom）
> 5. 配套launch文件结构、校验命令

## 6.1 先修正 URDF/Xacro 里Gazebo插件话题名（最关键）

打开 `robot_nav_description/urdf/robot.urdf.xacro`，修改3个gazebo插件的topic参数，严格匹配6.1表格 
 
## 6.2 robot_state_publisher：发布静态TF（base_link→laser、base_link→imu_link）

> 
> 职责：读取URDF，自动发布所有静态TF，对应表格：任意阶段 base_link→laser / imu_link，publisher=robot_state_publisher
> 放在 robot_nav_bringup/launch/robot_sim.launch.py

核心片段：

```
# robot_state_publisher
robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{'robot_description': robot_desc}],
    output='screen'
)
```

只要xacro里写好：

```
<link name="laser"/>
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser"/>
  <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
</joint>

<link name="imu_link"/>
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
</joint>

<link name="gps_link"/>
<joint name="gps_joint" type="fixed">
  <parent link="base_link"/>
  <child link="gps_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

robot_state_publisher 会自动发布这几组静态TF，**不用手动写static_transform_publisher**。

---

## 6.3 robot_localization（ekf_node）实现 odom -> base_link，输出 /odometry/filtered

> 
> 表格：建图/导航阶段，odom→base_link 由 robot_localization 发布；
> 输入：`/wheel/odom` + `/imu/data`
> 输出：`/odometry/filtered` + TF `odom → base_link`

新建 `robot_nav_bringup/config/ekf.yaml`

```
ekf_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true

    # 坐标系约定
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 输入源：轮速里程计
    odom0: /wheel/odom
    odom0_config: [true, true, false, false, false, false, false, false, false, false, false, false, false, false, false]
    odom0_relative: true
    odom0_queue_size: 10

    # 输入源：IMU
    imu0: /imu/data
    imu0_config: [false, false, false, true, true, true, false, false, false, false, false, false, true, true, true]
    imu0_relative: false
    imu0_queue_size: 10

    # 输出话题
    odom_out: /odometry/filtered
    publish_tf: true # ⭐ ekf 负责发布 odom -> base_link TF
    publish_acceleration: false
```

在launch里加载ekf：

```
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[os.path.join(config_dir, "ekf.yaml")],
    output='screen'
)
```

> 
> ⚠️ 重点：不要让diff_drive、其他节点发布odom→base_link TF，`publish_tf`只在ekf打开。

---

## 6.4 map → odom 的两个场景：建图阶段（SLAM Toolbox）/导航阶段（AMCL）

### 场景A：建图（SLAM Toolbox）

`robot_nav_slam/config/slam_toolbox.yaml` + slam launch

- slam_toolbox 负责**建图 + map -> odom TF**
- 话题 `/map` 由slam_toolbox发布（nav_msgs/OccupancyGrid）> 
> 对应表格：建图阶段 map -> odom publisher = slam_toolbox

### 场景B：导航（AMCL，已建好地图）

`robot_nav_bringup/config/amcl.yaml` + nav2 launch

- amcl 读取静态地图，输出 `map -> odom` TF
- `/map` 由 `map_server` 加载离线地图发布> 
> 对应表格：导航阶段 map -> odom publisher = amcl

> 
> 注意：**同一时刻只能二选一**，不能同时开slam_toolbox和amcl，两个都会发布map→odom，TF冲突。

---

## 6.5 完整TF树结构验证（和6.2一致）

```
map -> odom -> base_link
                ├─> laser
                ├─> imu_link
                └─> gps_link
```

校验命令，启动仿真+ekf后执行：

```
ros2 run tf_tools view_frames
# or
ros2 topic echo /tf
```

### 话题清单校验命令（核对6.1表）

```
# 看话题列表
ros2 topic list
# 查看消息类型
ros2 topic type /scan
ros2 topic type /wheel/odom
ros2 topic type /imu/data
ros2 topic type /cmd_vel
ros2 topic type /odometry/filtered
ros2 topic type /map
```

## 6.6 启动顺序建议（避免TF/时序报错）

1. Gazebo + spawn_entity（加载robot xacro模型）
2. robot_state_publisher（静态TF）
3. ekf_node（robot_localization，融合/wheel/odom + /imu/data）
4. 建图：slam_toolbox；导航：map_server + amcl + nav2
5. 可选：teleop_twist_keyboard（键盘发布 /cmd_vel）

## 6.7 常见踩坑点

1. ❌ gazebo diff_drive `publish_odom_tf=true` → 和ekf抢 odom→base_link TF，TF跳变
2. ❌ 里程计topic写成`/odom`，不是`/wheel/odom` → ekf收不到数据
3. ❌ ekf two_d_mode没开，IMU参数配置错误，融合漂移严重
4. ❌ frame名字大小写错误：`base_link` / `laser` / `imu_link` 必须完全一致，ROS TF大小写敏感
5. ❌ SLAM和AMCL同时启动，两个节点同时广播map→odom，TF冲突

# 配套下一部分：launch整合

你可以选择，我直接把**robot_sim.launch.py完整代码**一次性写出来，整合：

- 启动Gazebo
- 加载office.world
- spawn机器人模型
- robot_state_publisher
- ekf_node
- 可选键盘遥控

需要我直接生成这个完整launch文件吗？