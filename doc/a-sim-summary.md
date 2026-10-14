# ch01.仿真侧问题记录：定位退化、AMCL 初值与 Gazebo 稳定性

> 记录时间：2026-10-12
> 范围：ROS 2 Humble + Nav2 + `robot_nav_bringup` 仿真链路（办公室方形房间 5×5 m）
> 本文只记录**本次实测**遇到的问题、证据、根因与处置；未修的都明确标注为「未应用」。

---

## 0. 环境约定（先记下来，否则命令会白跑）

| 项 | 值 |
|---|---|
| 运行环境 | 容器 `humble_nav2`，**不是**主机 |
| 主机工程路径 | `/mnt/a2disk/a2/zdev/nv/adas-01/ros/ros2humble_nav_ws` |
| 容器内路径 | `/root/ros/ros2humble_nav_ws`（同一份代码的挂载） |
| 每次进入需 source | `/opt/ros/humble/setup.bash` + `install/setup.bash` |

**容器 `/tmp` 与主机 `/tmp` 不共享。** 给容器用的脚本必须 `docker cp` 进去；容器里产生的日志
（如 `/tmp/nav2.log`）要 `docker cp` 出来或直接打印到 stdout。本次所有诊断脚本都在主机
`/tmp/*.py`，`docker cp` 后运行。

本次的交付物（已在位）：

```
src/robot_nav_bringup/maps/demo_map.pgm    10216 B  101×101 @ 0.05  origin (-2.53,-2.54,0)
src/robot_nav_bringup/maps/demo_map.yaml   126 B    523 个占据格
src/robot_nav_bringup/maps/backup_symmetric_room/   旧对称图（99×99 @ 0.05, origin -2.48,-2.48)
```

---

## 1. 问题总览

| # | 问题 | 现象 | 根因 | 状态 |
|---|---|---|---|---|
| 1 | 方形房间定位退化 | AMCL 锁死在 90°/180° 镜像位姿；RViz 出现「假墙」 | 房间四壁对称，内部无不对称特征 | **部分解决**（加地标+重出图，余量仍不足，见 §2.1） |
| 2 | AMCL 无法从错误初值恢复 | 从 `0,0,0` 起步，信念稳定停在真值的 −90° 旋转像 | `recovery_alpha_slow/fast = 0.0`（无全局重定位）+ 初值离真值 ~1.8 m/143° | 未应用（需你决定） |
| 3 | 误判「`/initialpose` 不生效」 | 我一度认为种子没被接收 | **我的诊断方法错**：读到的是 seed 前锁存的旧 `/amcl_pose` | 已澄清（§2.3） |
| 4 | `gzserver` 中途崩溃 | 仿真死掉，`gzclient` 变孤儿，测试中断 | 容器 fd 上限 1024 被耗尽 | **待处理**（重启 sim 前加 `ulimit`） |
| 5 | `/clock` QoS 不兼容 | `requesting incompatible QoS ... RELIABILITY_QOS_POLICY` | 某节点以 RELIABLE 订阅，而 `gazebo_ros` 以 BEST_EFFORT 发布 | 未定位到具体节点 |
| 6 | costmap 膨胀半径不合理 | 两个 `inflation_layer` ERROR | `inflation_radius` < 内切半径 0.304 | 未应用 |
| 7 | 地图左墙双带 | 左墙出现 ~0.2 m 宽的重影 | **成因未证实**（原以为是 EKF yaw，实测数据不支持简单结论，见 §2.7） | 未定因 |

---

## 2. 逐项说明

### 2.1 方形房间的定位退化（本次的主问题）

**现象.** 空方形房间里，AMCL 会锁定到真实位姿的 90°/180° 旋转像上；RViz 里表现为「不该是障碍物
的地方出现直角墙」。

**根因.** 房间近似正方形、四壁对称，机器人在房间内任何位置取的扫描，在绕房间中心旋转 90° 的整数倍
后几乎同样能匹配上墙壁。对称性让似然函数出现 4 个近乎等高的峰。

**证据（配置无关的打分器）.** 用真实扫描，比较「真值位姿」与「真值位姿绕地图中心的 90/180/270°
对称像」（打分 = 每束端点到最近占据格的平均距离，越小越好；inliers = 距离 < 0.15 m 的束占比）：

| 地图 | 真值位姿 | 最佳对称像 | 比值 |
|---|---|---|---|
| 旧（对称房） | 0.0606 m / 91.1% | 0.0667 m / 90.0% | **1.10** |
| 新（加地标） | 0.0293 m / 95.0% | 0.0407 m / 93.3% | **1.39** |

旧图四个盆分别是 91.1% / 90.0% / 88.3% / 88.9% —— **几乎完全等价**，这就是「假墙」的量化形态。
新图对 +90/+270 的余量是 2.2–2.5×，对 +180 只有 1.39×。

**已做的处置.**

1. 在 world 里加两件不对称地标：`pillar_1`（0.3 m 方柱，真值 (1.50, 1.20)）、`cabinet_1`
   （0.6×0.35×0.45 柜子，真值 (−1.70, −0.90)）。
2. 重新建图并覆盖 `maps/demo_map`（`nav2_map_server map_saver_cli -f ... -t /map`）。
   新图里两个地标确实入图：pillar 16 格、质心 (1.479, 1.119)；cabinet 23 格、质心 (−1.518, −0.758)
   （cabinet 只有侧面可见，质心偏 ~15 cm 属正常）。
3. 新图内部不再 180° 自相似：180° 旋转下内部障碍 0/40 匹配（改前是全匹配）。

**为什么只能说「部分解决」——量化原因.** 在真值位姿下：

```
360 束里只有 3 束打在地标上（0.8%）
AMCL 的 max_beams: 60 抽样后，只剩 2 束
```

也就是说整张图的不对称信息只由 **2/60 束**承载（墙占掉另外 58 束），likelihood field 很难仅凭
这 2 束压倒镜像假设。在 +180 镜像位姿上，「解释得很差（>0.15 m）」的束只有 25 束，抽样后仅 4 束。

**建议（未应用）.**

- 想让不对称真正生效，地标必须**大得多或更多**：在 2–3 m 观测距离上要张角几十度，即 ~0.8–1.0 m
  宽的板/柱，或布置 4–6 个。0.3/0.6 m 的小物体在当前 `max_beams: 60` 下信号太弱。
- 另一个方向是把初始位姿交给操作者（RViz 的 2D Pose Estimate，见 §2.3）+ 打开重定位（见 §2.2）。

### 2.2 AMCL 从错误初值无法恢复（配置陷阱）

**现象.** sim 起来后 AMCL 以 `set_initial_pose: True` / `initial_pose: 0,0,0`
（`config/nav2_params.yaml:9-10`）起步，而车实际停在离原点约 1.8 m 处。AMCL 的信念没有收敛到真值，
而是稳定停在真值的 **−90° 旋转像**上。

**证据.**

```
gz 真值    = ( -1.174,  1.149,  -52.85°)
AMCL 信念  = (  1.146,  1.164, -144.43°)
把真值绕原点转 -90° -> (1.149, 1.174, -142.85°)
                             差 5 mm / 1.6°      <- 精确落在对称像上
map->odom  = ( -0.068,  0.042,  -92.89°)
```

**根因.** 两件事叠加：

1. `recovery_alpha_slow: 0.0`、`recovery_alpha_fast: 0.0`（`nav2_params.yaml:43-44`）
   → 不注入随机粒子 → **没有全局重定位能力**。粒子只能从初值出发沿似然爬坡，无法跳出最近的
   对称盆。
2. 初值本身错得远（`0,0,0` 与真值差 ~1.8 m / 143°），而房间里 4 个对称盆离初值几乎等距，
   「选哪一个」基本由局部似然决定。

**建议（未应用，等你决定）.**

- 操作侧：用 RViz 的 **2D Pose Estimate** 正确给初值（这就是现行流程，已验证有效，见 §2.3）。
- 配置侧：把 `recovery_alpha_slow` 设为 0.001–0.05 量级（`recovery_alpha_fast` 通常给 0.1 左右），
  让 AMCL 具备从错误位姿爬出来的能力；代价是位姿会偶尔抖动，需要重新调 `alpha1..5`。
- 顺带：`set_initial_pose: True` 的 `initial_pose` 若保持 `0,0,0`，请确保**车就是出生在原点**，
  否则每次启动都从错位姿开始，在对称房间里必然踩坑。

### 2.3 「`/initialpose` 不生效」是我的误判（诊断方法问题）

**经过.** 我先用脚本发布 `/initialpose`（RViz 等价），随后读 `/amcl_pose` 发现位姿像没变，
一度判断「种子没被接收」，并去怀疑 QoS。

**真正原因（两个叠加的读取陷阱）.**

1. `nav2_amcl` 发布 `/amcl_pose` 用的是 **transient_local（锁存）** QoS；普通 volatile 订阅要么收不到，
   要么拿到的是**上一次**的旧值。
2. AMCL 只在**移动超过** `update_min_d: 0.25` m 或 `update_min_a: 0.2` rad
   （`nav2_params.yaml:51-52`）后才跑一次滤波更新。停着的车（本机还有 ~1.2 cm/s 的蠕动）根本不触发更新，
   所以 `/amcl_pose` 一直是旧的。

**排除项.** QoS 不兼容**不是**原因 —— `ros2 topic info -v /initialpose` 显示 amcl 端订阅是
**BEST_EFFORT / VOLATILE**，RViz 端发布是 RELIABLE / VOLATILE，二者兼容（我自己的发布器也是兼容的）。

**正确做法（已验证）.**

1. 订阅 `/amcl_pose` 时把 durability 设成 `TRANSIENT_LOCAL`。
2. 判断种子是否生效，不要只看位姿，先让它动 ~0.3 m 强制一次更新；或者直接看 AMCL 日志。
3. 最硬的证据是日志（本次 30 条消息逐条都有）：

```
[amcl]: initialPoseReceived
[amcl]: Setting pose (2464.000000): -1.174 1.149 -0.922
```

（日志里伴随的 `Failed to transform initial pose in time (Lookup would require extrapolation into
the future ...)` 是**无害**的：AMCL 回退后照样把位姿设好了，本次实测种子与信念差 8 mm / 0.2°。）

**结果.** 种子生效后直行 0.414 m，跟踪误差 0.109 m / 0.9° —— 正常。

### 2.4 `gzserver` 因文件描述符耗尽崩溃

**现象.** 满圈跟踪测试中途仿真直接死掉（`gz model -m nav_robot -p` 返回
`An instance of Gazebo is not running.`），`gzclient` 变成孤儿进程。

**证据（launch 日志原文）.**

```
[gzserver-1] getifaddres: Too many open files
[gzserver-1] terminate called after throwing an instance of 'gazebo::common::Exception'
[ERROR] [gzserver-1]: process has died [pid 10020, exit code -6, cmd 'gzserver .../office.world.sdf ...']
```

`exit code -6` 即 SIGABRT。

**根因.** 容器内 `ulimit -Sn` = **1024**（`-Hn` = 524288）。`gzserver` 的 transport
（socket、接口枚举）在长跑中不断累积 fd，一旦触及 1024 就抛异常自杀。本次反复起短命 `rclpy` 节点、
又多次调用 `gz model` CLI，每次都新增 DDS/transport 参与者，**极可能是压死它的直接原因**。

**处置（待重启 sim 时执行）.**

```sh
# 进入容器后先抬高上限，再启动仿真
ulimit -n 65536
ros2 launch robot_nav_bringup simulation.launch.py
```

或在容器创建时固化：`docker run --ulimit nofile=65536 ...`。
另外**减少诊断脚本的启停次数**：一个脚本里把「快照 + 打分 + 打印」做完，别一束一束地起节点。

### 2.5 `/clock` QoS 不兼容

**现象.**

```
[gzserver-1] [WARN] [gazebo]: New subscription discovered on topic '/clock',
  requesting incompatible QoS. No messages will be sent to it.
  Last incompatible policy: RELIABILITY_QOS_POLICY
```

**根因.** 有节点以 RELIABLE 订阅 `/clock`，而 `gazebo_ros` 以 BEST_EFFORT 发布；可靠性策略不兼容时
DDS 直接不投递 → 该节点永远拿不到仿真时间（`use_sim_time: true` 下时间停在 0，定时器/超时会错乱）。

**处置（未定位）.** 下次起 sim 时用 `ros2 topic info -v /clock` 抓订阅方，把它的 QoS 改成
BEST_EFFORT（一般还要 `TRANSIENT_LOCAL` + depth 1，才能在启动时立刻拿到当前时间）。

### 2.6 costmap 膨胀半径小于内切半径

**现象（启动即报的 ERROR，两个 costmap 各一条）.**

```
[local_costmap.local_costmap]: The configured inflation radius (0.050) is smaller than the
  computed inscribed radius (0.304) of your footprint, it is highly recommended to set
  inflation radius to be at least as big as the inscribed radius to avoid collisions
```

**根因.** `robot_radius: 0.30`（`nav2_params.yaml:217`、`:265`）→ 内切半径 0.304；
但 `inflation_radius` 只给了 0.05（local，`:231`）和 0.12（global，`:297`），而
`cost_scaling_factor: 3.0`（`:230`、`:296`）。膨胀半径比机器人自身轮廓还小，等于**没有膨胀**，
规划器会贴着墙走。

**建议（未应用）.** 两个 costmap 的 `inflation_radius` 提到 ≥ 0.304（实践上取 0.35–0.45），
`cost_scaling_factor` 相应下调（可从 3.0 降到 1.5–2.0 试），否则代价衰减过陡、通道会被堵死。

### 2.7 地图左墙双带 / odometry 与 EKF（成因未证实）

**现象.** 新图左墙 `x ∈ [−2.48, −2.28]` 有一道 ~0.2 m 宽的重影（连通域分析里表现为左墙分裂成
95 格 + 1 格的多个分量）。这是目前新图唯一的明显缺陷。

**实测数据（本次）.**

```
odom->base_link = ( -1.174,  1.148,  -50.78°)
gz 真值          = ( -1.174,  1.149,  -52.85°)
                   -> 位置差 1 mm，yaw 差 2.07°
```

**重要更正.** 之前我记录过「`ekf.yaml` 既没融合 odom 的绝对 yaw 也没融合 IMU 的绝对 yaw」——**这是错的**。
按 robot_localization 的 15 元素顺序（x,y,z, roll,pitch,yaw, vx,vy,vz, vroll,vpitch,vyaw, ax,ay,az）实读：

- `odom0_config`（`config/ekf.yaml:15-19`）：yaw（第 6 位，第 16 行）**= false**，vyaw（第 12 位）**= true**
- `imu0_config`（`ekf.yaml:24-28`）：yaw（第 6 位，第 25 行）**= true**，vyaw **= true**
- 但 `imu0_differential: true`（`ekf.yaml:29`）→ IMU 的 yaw 只以**增量**形式参与
- `world_frame: odom`（`ekf.yaml:12`）、`two_d_mode: true`（`:6`）→ 这个 EKF 的职责本就是跟踪
  odom，不需要绝对 yaw 锚定（map→odom 由 AMCL 负责）

**结论.** 配置本身没有「缺绝对 yaw」这种错误；实测 odometry 位置误差只有 1 mm。左墙双带是否由
yaw 引起，**目前没有证据**，需要专门实验（例如建图时同步记录 `/tf` 与 Gazebo 真值对比）才能定因。
不要按旧结论去改 `ekf.yaml`。

### 2.8 诊断工具自己的坑（避免重犯）

1. **占据栅格约定.** `nav2_map_server`：`occ = (255 − pixel)/255`，若 `negate` 再翻转 → **深色像素才是占据**。
   我一开始弄反，把扫描对到了空白格上，得出的「退化」结论全是假的（当时还表现为
   `ZeroDivisionError` / 「using 0 of 360 beams」/ 占据格数明显不对）。改对后占据格 523，与独立统计一致。
2. **退化测试必须旋转整个位姿.** 判断「房间是否退化」，要把**真值位姿整体绕地图中心旋转**再看匹配，
   而不是在原地只改 yaw。原地转 yaw 时 2.45 m 外的墙根本对不上，任何地图都会「轻松获胜」，
   得出假阳性（我第一版就是这么错的：比值 11.3 / 5.3，修正后才是 1.39 / 1.10）。
3. **均值距离指标 ≠ AMCL 的判决.** 打分器的「平均距离」会被 ~168 束墙稀释；AMCL 用的是
   likelihood field，**逐束似然相乘**，对少数关键束敏感得多。两者结论可能不一致（本次新图在打分器上
   真值胜出 2.2–2.5×，但 AMCL 从错初值出发仍落进对称像）。**不要用打分器代替端到端 AMCL 判决。**
4. **打分器要自检.** 用同一张图做一次合成扫描（从已知位姿射线投射），确认真值盆得分≈0 且胜出，
   否则先怀疑工具，而不是先怀疑地图。
5. **`pkill -f` 的括号技巧会自匹配.** `pgrep -f "[a]sync_slam_toolbox_node"` 的**命令行本身**含有
   `slam_toolbox` 子串（`[a]sync_slam_toolbox_node` 里包含 `slam_toolbox`），如果同一命令行里还有别的
   匹配模式，会把包装脚本自己也算进去。**要单独开一个 `docker exec`，命令行里只出现括号形式**。
6. **前台 `sleep`.** 顶层 `sleep` 会被拦；用 `run_in_background`，或写成「条件轮询循环」
   （`until grep -q ...; do sleep 2; done`）。
7. **`python3 -I` 对 ROS 脚本不适用**（隔离模式切掉 site-packages，`import rclpy` 直接失败）。

---

## 3. 当前状态与未决项

**已在位、可复现**

- 新图 `maps/demo_map`（101×101 @ 0.05，origin −2.53/−2.54）已覆盖，旧图完整备份在
  `maps/backup_symmetric_room/`。
- 「操作者给正确初值 → AMCL 跟踪」这条链路是通的：种子误差 8 mm / 0.2°，直行 0.414 m 后
  0.109 m / 0.9°。

**未决**

| 项 | 说明 |
|---|---|
| 满圈跟踪测试 | **未完成** —— `gzserver` 在测试中途崩溃（§2.4），需要在抬高 `ulimit` 后重跑 |
| 地标尺寸 | 当前 3/360 束（2/60 抽样束）承载不对称，余量偏薄；是否加大/加多需你定（§2.1） |
| `recovery_alpha_*` | 保持 0.0 则 AMCL 永远无法从错初值恢复；是否打开需你定（§2.2） |
| `inflation_radius` | 两条 ERROR 仍在线（§2.6） |
| `/clock` QoS | 未定位到具体订阅方（§2.5） |
| 左墙双带 | 成因未证实，不要按旧的 EKF yaw 结论去改（§2.7） |

---

## 4. 附录：本次使用的脚本与命令

诊断脚本（主机 `/tmp/`，`docker cp` 进容器后运行）：

| 脚本 | 作用 |
|---|---|
| `snap.py` | 原子快照一帧 `/scan` + Gazebo 真值 → `snap.json` |
| `synth_scan.py` | 从地图按已知位姿射线投射出合成扫描（给打分器做自检） |
| `likelihood.py` | 配置无关的退化打分器：真值位姿 vs 其 90/180/270° 对称像 |
| `landmark_beams.py` | 统计有多少束真正打在地标上、其中多少进了 AMCL 的 60 束抽样 |
| `amcl_spin.py` / `amcl_seed.py` / `amcl_seed2.py` | 原地旋转 / 给种子后跟踪 / 种子接收证明（`amcl_seed2.py` 会先用直行 0.4 m 强制一次更新） |

常用命令：

```sh
# 进容器 / 传文件
docker exec -it humble_nav2 bash
docker cp /tmp/likelihood.py humble_nav2:/tmp/

# 存图（-t 是话题，-f 是输出文件前缀）
ros2 run nav2_map_server map_saver_cli -f maps/demo_map -t /map

# 看 /initialpose 两端 QoS（本次用来排除 QoS 不兼容）
ros2 topic info -v /initialpose

# 订阅锁存的 /amcl_pose 时必须带 TRANSIENT_LOCAL，否则读到旧值
```

Gazebo 真值（比 `/odom` 可靠，用于一切对比）：

```sh
gz model -m nav_robot -p     # 输出: x y z roll pitch yaw
```

=========================================
=========================================



IMU 的 yaw（第 6 位）确实是 true。只是 imu0_differential: true（:29）让它只以增量参与，且 world_frame: odom（:12）本来就不需要绝对 yaw 锚定（map→odom 归 AMCL 管）。所以「缺绝对 yaw」不是配置错误，左墙双带也没有证据指向 yaw —— 实测 odom→base_link 与真值位置只差 1 mm。这条我在文档里标成了「成因未证实」，并写明不要按旧结论去改 ekf.yaml。

文档结构（中文，跟 nav2-robot-1week-guide/a-sim-claude.md 同一体例，每条都带 现象/证据/根因/方案/状态）：

┌──────────┬───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│   章节   │                                                                               内容要点                                                                                │
├──────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ §0       │ 容器环境约定（主机↔容器路径、/tmp 不共享需 docker cp）、本次交付物                                                                                                    │
├──────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ §1       │ 7 项问题总览表                                                                                                                                                        │
├──────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ §2.1     │ 方形房间退化：旧图四盆 91.1/90.0/88.3/88.9% 近乎等价（比值 1.10）→ 新图 95.0% vs 93.3%（比值 1.39）；3/360 束、抽样后仅 2/60 束承载不对称，这是「部分解决」的量化理由 │
├──────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ §2.2     │ AMCL 初值陷阱：set_initial_pose: 0,0,0 + recovery_alpha_slow/fast = 0.0（:9-10,:43-44）；实测信念精确落在真值 −90° 旋转像上（5 mm/1.6°）                              │
├──────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ §2.3     │ 我误判「种子无效」的两个读取陷阱（/amcl_pose 锁存 + update_min_d/a 不触发更新），并记录 QoS 已于排除                                                                  │
├──────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ §2.4     │ gzserver fd 耗尽（ulimit -Sn 1024）→ SIGABRT，含修复命令                                                                                                              │
├──────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ §2.5–2.7 │ /clock QoS、inflation_radius 0.05/0.12 < 内切 0.304、左墙双带（未定因）                                                                                               │
├──────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ §2.8     │ 诊断工具自身的坑（占据栅格深浅、必须整体旋转位姿而非原地转 yaw、均值距离≠AMCL 逐束似然、pkill -f 自匹配、前台 sleep、python3 -I 对 ROS 无效）                         │
├──────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ §3–4     │ 未决项清单 + 脚本/命令附录                                                                                                                                            │
└──────────┴───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

顺带把那个卡住的后台测试（amcl_seed2 满圈测试）停掉了 —— 它停在 "spinning a full turn"，而 sim 已经死了，不会再有结果。当前 gzserver 仍是死的，gzclient 是孤儿进程，其余节点还在跑；重跑满圈测试需要重启 sim，这一点我按你的意思没有动。

✻ Brewed for 2m 9s · done 8:55 PM

※ recap: Goal was documenting the sim localization problems; doc/a-sim.md is now written with all seven issues, evidence and fixes. Next action is yours: restart the sim if you want the full-turn AMCL test rerun. (disable recaps in /config)
 