# ch02.仿真侧问题记录：导航 abort 循环、AMCL 似然多峰与几何/参数修正

> 记录时间：2026-10-14
> 范围：ROS 2 Humble + Nav2 + `robot_nav_bringup`（办公室方形房间 5.05×5.05 m）
> 承接 `ch01`（`doc/a-sim-summary.md`）。本文只记录**本次实测**遇到的问题、证据、根因与处置；
> 未修的都明确标注为「未应用」。

---

## 0. 承接 ch01 的未决项

| ch01 编号 | 问题 | ch02 结论 |
|---|---|---|
| ch01-1 | 方形房间定位退化（90°/180° 镜像） | **机理确认并升级为本文主问题**：不是"似然太平"，而是**多个几乎等高的尖峰**，见 §2.1 |
| ch01-2 | AMCL 无法从错误初值恢复 | 该配置项已修（§2.5），但**单独修它不够**——全局定位仍 5/5 失败，见 §2.1 |
| ch01-6 | costmap 膨胀半径 < 内切半径 | 已改为 0.35（§2.6），**但这一改未经用户批准**，见 §4 |
| ch01-4 | `gzserver` fd 耗尽崩溃 | 本次未复发 |

---

## 1. 问题总览

| # | 问题 | 现象 | 根因 | 状态 |
|---|---|---|---|---|
| 1 | **导航 abort 循环**（主问题） | 每 ~10 s 一次 `Failed to make progress` → abort → 清 costmap → 新目标 | AMCL 定位错 **2.28 m / 65°** 却高置信（`cov[0]=0.3207`），bt_navigator 在不存在的位置上规划 | 机理确认，**未修** |
| 2 | 似然函数为何锁错 | 全局定位 5/5 失败；宽云直接放在真值上仍漂走 2.14 m | 房间四重旋转对称 → 2 m 外存在与真峰只差 **1–4%** 的镜像尖峰；`min_particles=500` 的重采样噪声 ≈ **4.5%** > 峰间差 | 机理确认，**未修** |
| 3 | DWB 恒以为车速为 0 | 轨迹可达性与 `RotateToGoal` 停转判定建立在"当前速度=0"的假前提上 | `controller_server.odom_topic` 默认 `"odom"`，本工程 `/odom` **无任何发布者** | **已修**（§2.4） |
| 4 | 脚轮与驱动轮半径不一致 | 车体永久俯仰 1.91°，扫描面擦到自身结构 | 脚轮 `cylinder r=0.04`，驱动轮 `r=0.05` | **已修**（§2.7） |
| 5 | AMCL 随机粒子恢复路径永久关闭 | 无全局重定位能力 | `recovery_alpha_fast/slow` 都是 0 → `w_diff = 1 - w_fast/w_slow` = 0/0 = **NaN** | **已修**（§2.5） |
| 6 | costmap 膨胀半径不合理 | 两个 `inflation_layer` ERROR | `inflation_radius` 0.05/0.12 < 内切半径 0.304 | **已改但未获批准**（§2.6） |
| 7 | 地标压不住简并 | 镜像位姿仍能与真值竞争 | 地标太小（0.3×0.3 / 0.6×0.35） | **已放大并实测**（§2.3），有效但未解决 |
| 8 | 我自己的三个错误假设 | 差点按错方向改代码 | 见 §3 | 已由测量否掉 |

---

## 2. 逐项说明

### 2.1 导航 abort 循环（本次的主问题）

**现象.** bt_navigator 反复规划、控制器反复 `Failed to make progress` 并 abort，随后清空
local_costmap、换新目标、再失败。周期约 10.5–11 s。

**复现到的关键快照.** 同一时刻两路数据：

| 来源 | x | y | yaw |
|---|---|---|---|
| `/wheel/odom`（Gazebo 真值） | **+1.515** | **+2.209** | **+176.61°** |
| `/amcl_pose` | +0.7877 | +0.0486 | ≈+111.3° |

误差 **2.28 m / 65°**，而 `covariance[0] = 0.3207` —— **错得很自信**。
bt_navigator 的日志与之吻合：`Begin navigating from current location (-0.10, -0.01) to (1.60, 1.60)`，
而机器人实际在 (1.293, 1.442)。

**排除"机器人不会动".** 用 `/wheel/odom`（本工程为 WORLD 真值）开路实测：

| 指令 | 实测 | 结论 |
|---|---|---|
| 直行 +0.150 m/s | 0.144 m/s（96%），航向漂 −11.1° | 正常 |
| 后退 −0.150 m/s | 0.151 m/s（101%） | 正常 |
| 自转 +0.45 rad/s | +0.405 rad/s（90%） | 正常 |
| RTF | 1.00 | 正常 |

**并且**：把 AMCL 紧种子到真值，它跟踪到 **0.06–0.10 m**，到点全部成功。

**根因.** 问题不在运动、不在初始化，而在**似然函数本身锁错**。见 §2.2。

### 2.2 似然的真实形状：多个几乎等高的尖峰（本次最重要的测量）

**先说结论：我原先"似然太平"的判断是错的。** 真实地图 + 真实 `/scan`，用 AMCL 的精确公式
（`pz = z_hit·exp(−z²/2σ²) + z_rand/range_max`，位姿得分 `= Σ pz³`，`z = occ_dist(束端点)`），
理论上限 `1 + 120×(0.9+0.1/8)³ = **92.18**`。

沿真值位姿逐轴扫描（`offset.py` / `profile.py`）：

```
真峰                            78.4 – 81.0    (85 – 88% of 92.18)
偏离 0.20 m                     36 – 55         (39 – 60%)     ← 梯度其实很陡
峰宽   x ≈ ±5 cm,  yaw ≈ ±2.5°
2.06 m 外 (+0.17,−1.44,−84.0°)  77.50          ← 与真峰只差 1–4%
里程计位姿本身                   69.57          (75.5%)
```

**所以是"多尖峰"，不是"平台"。** `office.world.sdf` 是 5.05 m 正方形封闭房间，四重旋转对称造出了
2 m 外的第二/三/四个几乎等高的尖峰。

**为什么噪声会赢过信号.**

| 量 | 值 |
|---|---|
| 峰间差 | **1 – 4%** |
| `min_particles = 500` 时的重采样噪声 | `1/√500` ≈ **4.5%** |

噪声大于信号，粒子云可以在峰间跳；跳过去之后 `map→odom` 就恒定错约 2 m。

**这条机理同时解释了三件事：**

1. **全局定位 5/5 失败** —— `/reinitialize_global_localization` 后粒子均匀铺满地图，转 15 s，
   五轮全部收敛到同一错误区域，误差 2.39 / 2.52 / 2.47 / 2.35 / 2.41 m。
2. **宽云直接放在真值上也漂走** —— 以真值为中心、sd = 1.41 m / 41° 的宽云，最终漂到 **2.14 m** 误差。
   把云放在吸引子中心则稳定停在吸引子：**似然自己就偏好错误位姿**。
3. **紧种子正常** —— 云在正确的盆地里，梯度把它按住。

**顺带否掉的两个人造解释：**

- **"地图与世界没对齐"** —— 否掉。地图自身最优位姿离里程计位姿只有 **3.6 cm**（< 1 个栅格），
  且不随搜索窗口（0.10 / 0.25 / 0.50 / 1.00 m）变化。
- **"缩短雷达量程就能解决"** —— 否掉，见 §3.2。

**未修.** 方向见 §4。

### 2.3 地标放大：有效但未解决

上一次会话已把 `office.world.sdf` 的两个地标放大并重新选址，同步改了 `maps/demo_map.pgm`：

| | 原 | 现 |
|---|---|---|
| 柱子 | `0.3×0.3` @ (1.50, 1.20) | `0.7×0.7` @ (1.25, −0.55) |
| 柜子 | `0.6×0.35` @ (−1.70, −0.90) | `1.2×0.5` @ (−0.65, −1.50) |

实测效果（记在 `office.world.sdf` 注释里）：用真实 `/scan` 反投影，真值与最佳镜像只差
**0.13 m/beam**（`sigma_hit=0.1`）；全工作空间 **12.7%** 的可达位姿上"镜像比真值更优"，最差
**−5.0 nats**。放大后对手胜出降到 **3.4%**，中位对数似然裕度 **6.5 → 17.1 nats**。

**结论：有效，但 3.4% 不等于 0。** 与 §2.2 的独立测量吻合——多峰问题被削弱但没有根除。

### 2.4 DWB 恒以为车速为 0（已修）

**现象.** `dwb_core::DWBLocalPlanner` 的轨迹可达性判断、加速度约束、`RotateToGoal` 的停转判定
全部建立在"当前速度 = 0"这个假前提上。

**根因.** `nav2_controller` 的 `ControllerServer` 声明 `odom_topic` 的默认值是 `"odom"`，
而本工程 **`/odom` 没有任何发布者**（只有 `controller_server` 自己订阅它）。
`nav2_util::OdomSmoother` 于是永远收不到消息，报出的底盘速度恒为 0。
`bt_navigator` 早就指向 `/odometry/filtered`，两边不一致。

**处置（已应用）.** `nav2_params.yaml`：

- `controller_server.ros__parameters` 增加 `odom_topic: /odometry/filtered`
- `velocity_smoother` 的 `odom_topic: "odom"` → `/odometry/filtered`

> 注：`velocity_smoother` 用 `OPEN_LOOP` 反馈时不会暴露这个问题，只有切到 `CLOSED_LOOP` 才会。

### 2.5 AMCL 随机粒子恢复路径永久关闭（已修）

**根因.** `recovery_alpha_fast` 与 `recovery_alpha_slow` 都是 `0.0` 时，
`w_diff = 1 − w_fast/w_slow` 恒为 `0/0 = **NaN**`，随机粒子注入路径被永久关闭。

**处置（已应用）.** `recovery_alpha_fast: 0.0 → 0.1`，`recovery_alpha_slow: 0.0 → 0.001`。

> 但 §2.2 已证明：**单靠这一项救不回来**，因为全局定位失败的原因是似然多峰，不是搜索不够。

### 2.6 costmap 膨胀半径（已改，但**未经批准**）

**根因.** `inflation_radius` 小于机器人内切半径 0.304，`inflation_layer` 报 ERROR。

**处置（已改，待用户裁决）.**

| 参数 | 原 | 现 |
|---|---|---|
| `local_costmap.inflation_layer.inflation_radius` | 0.05 | 0.35 |
| `global_costmap.inflation_layer.inflation_radius` | 0.12 | 0.35 |
| 两者 `cost_scaling_factor` | 3.0 | 1.5 |

> **这一批改动不是用户批准的**，与 `max_beams: 60→120`、`recovery_alpha_*` 一样属于前序会话遗留。
> 若后续要继续改 `nav2_params.yaml`，必须声明"覆盖了这些未批准的改动"。

### 2.7 脚轮与驱动轮半径不一致（已修）

**根因.** 脚轮是 `cylinder radius="0.04"`，驱动轮 `r=0.05`。差 1 cm 使车体产生
`atan(0.01/0.30) ≈ 1.91°` 的**永久俯仰**，扫描平面因此擦到自身结构，打出 0.26–0.55 m 的假回波。

**处置（已应用）.** 脚轮改为 `sphere radius="0.05"`（与驱动轮同半径），
惯性宏同步改为 `inertial_sphere`（`ixx=iyy=izz=0.4·m·r²`）。

### 2.8 `nav2_amcl` 没有里程计订阅（排查时的关键认识）

AMCL **不订阅任何 odom 话题**，它的运动模型完全来自 TF（`odom_frame_id` → `base_frame_id`），
即 EKF 的 `odom→base_link`。本工程 `/wheel/odom` 是 Gazebo 真值，EKF 以
`odom0_differential: false` / `odom0_relative: false` 绝对融合 x,y，所以 `odom→base_link`
是平滑过的真值——**AMCL 的运动模型是理想的**。所以"AMCL 缺里程计"这条假设不成立。

---

## 3. 我自己的错误假设（避免后人重犯）

这三条都是我先提出、再被自己的测量否掉的，记下来是为了不让它们以后被当成结论。

### 3.1 「AMCL 定位错了，因为房间太对称导致似然太平坦」

**半错.** 房间确实四重对称，但似然**不平**——真峰 85–88%，偏离 20 cm 就掉到 39–60%，
峰宽只有 ±5 cm / ±2.5°。正确的说法是**多个尖峰**，不是平台。这个区别决定了修法：
"平台"要改传感器模型或加特征，"多峰"要**打破旋转对称**或**降重采样噪声**。

### 3.2 「缩短雷达有效量程可以让束在错误位姿处死在空旷处」

**否掉.** `fixprobe.py` 对 826,680 个位姿（5 cm × 5 cm × 3°）做精确全局 argmax，
扫描有效量程 8.0 / 5.0 / 4.0 / 3.0 / 2.5 / 2.0 / 1.5 / 1.2 m：

```
max_rng  beams   truth  best-else   margin
    8.0    120   69.57     77.50     0.898
    5.0    120   69.57     77.50     0.898
    4.0    100   58.00     66.80     0.868
    3.0     93   53.62     61.48     0.872
    2.5     73   43.25     51.64     0.838
    2.0     59   37.89     41.24     0.919
    1.5     49   31.34     36.04     0.870
    1.2     37   22.93     29.11     0.788
```

**每一档 margin 都 < 1**，对手（(+0.17,−1.44,−84.0°)，2.06 m 外）每一档都赢。改动无效。

### 3.3 「地图和世界没对齐」

**否掉.** 见 §2.2：最优位姿只差 3.6 cm。

### 3.4 诊断工具自身的坑

- **`whyflat.py` 早期的 "live attractor" 行无效**：用活体吸引子位姿去打分一张**在别处**采的
  `snap.json` 扫描。结论作废，不能引用。
- **`design.py` 的 ray-marcher 只有 83.3% 保真**（对真实扫描：中位误差 0.060 m、均值 0.571 m、
  p90 0.713 m），其自身会打印 "MODELLING ERROR, do not trust the designs below"。
  所以**房间形状的结论是初步的**；而用真实扫描 + 真实地图算出的饱和/多峰数字是可信的。

---

## 4. 当前状态与未决项

### 4.1 已应用（工作区，未提交）

| 文件 | 改动 |
|---|---|
| `config/nav2_params.yaml` | `controller_server.odom_topic`、`velocity_smoother.odom_topic` 指向 `/odometry/filtered` |
| `urdf/robot.urdf.xacro` | 脚轮 `cylinder r=0.04` → `sphere r=0.05`，新增 `inertial_sphere` 宏 |
| `worlds/office.world.sdf` | 两个地标放大并移位 |
| `maps/demo_map.pgm` | 与地标同步（备份 `maps/demo_map.pgm.bak-20261014-154331`） |

### 4.2 未获批准、需要用户裁决

`nav2_params.yaml` 中：`inflation_radius` 0.05→0.35 / 0.12→0.35、`cost_scaling_factor` 3.0→1.5、
`max_beams` 60→120、`recovery_alpha_fast/slow` 0→0.1/0.001。

### 4.3 未完成

1. **端到端复测从未跑完。** `goal_seq.py`（五点巡游）被 kill，只输出了表头。
   所有修复都还没经过"跑一遍到点"的验证——这是最关键的缺口。
2. **多峰根因未修。** 候选方向与实测效果：

   | 方向 | 测量值 | 备注 |
   |---|---|---|
   | 打破旋转对称（房间改非正方形） | 6.4×4.4 → 90° 别名压到 **×8.8**；4.6×6.4 → **×10.3**；东侧加门洞把最近竞争者 ×1.231 → **×1.394** | design.py 保真度仅 83.3%，**初步结论** |
   | 降重采样噪声 `resample_interval: 1 → 2` 或 `min_particles: 500 → 1000` | 未测 | 直接针对 4.5% vs 1–4% 的失配；**只调参数、可回滚、不碰世界几何** |
   | 收紧更新门限（现 `update_min_d: 0.25` / `update_min_a: 0.2`，每 25 cm 才纠一次） | 未测 | 未测 |

3. 前序遗留的低优先级问题：前进方向轮滑（`/wheel/odom` 报 vx 0.110 vs 真值 0.144，低 24%，
   但后退方向完全吻合；驱动轮无 `<gazebo reference>` 摩擦块）；`/imu/data` 无输出但 `imu0` 融合了
   其 yaw；`/odometry/filtered` 实际 ~10 Hz 而 `ekf.yaml` 写 `frequency: 50.0`。

---

## 5. 附录：本次使用的脚本与命令

诊断脚本都在**主机** `/tmp/nav-diag/`，`docker cp` 进容器运行。

| 脚本 | 作用 |
|---|---|
| `whyflat.py` | 测束端点分布：真值处 88.1% 上限、平均束端点离障碍 6.5 cm（栅格 5 cm）、仅 4.2% 的束有信息量 |
| `fixprobe.py` | 826,680 位姿精确全局 argmax + 有效量程扫描；**否掉了"缩短量程"** |
| `offset.py` | 地图最优位姿 vs 里程计位姿的偏移；**否掉了"地图与世界不对齐"**（仅 3.6 cm） |
| `profile.py` | 沿 x / y / yaw 扫描似然剖面；**证明是"多尖峰"而非"平台"** |
| `goal_seq.py` | 五点巡游端到端测试（**被 kill，未完成**） |
| `drive_test.py` | 开路驱动测试（前进/后退/自转），确认底盘运动正常 |
| `globalloc.py` | 全局定位测试（重初始化 + 自转） |
| `design.py` | 离线房间设计测试台（含 ray-marcher，保真度 83.3%） |

**环境与工具坑（本次新踩到的）**

```bash
# 容器内 /tmp 与主机 /tmp 不共享：脚本要 docker cp 进去
# 容器里跑任何东西前都要 source
source /opt/ros/humble/setup.bash && source /root/ros/ros2humble_nav_ws/install/setup.bash

# 后台任务里 python 的 stdout 是块缓冲，必须 -u，否则什么都看不到
python3 -u /tmp/nav-diag/profile.py

# pkill -f '<pattern>' 会匹配到自己，用方括号转义
pkill -f '[b]ringup_launch.py'

# colcon --symlink-install：install 空间的 urdf/config 是源文件的符号链接，改完即生效
```

**TF 预热竞态.** `lookup_transform('map', 'base_link', Time())` 在刚启动时会失败约 2 s
（`tf2_echo` 会先打印 `Invalid frame ID "map" ... frame does not exist`），
单次 spin 就取值的脚本会拿到 `None`。需要先空转 2 s。

**`/amcl_pose` 是 TRANSIENT_LOCAL 且只在滤波器真正运行时发布**（需要 `update_min_d`/`update_min_a`
的位移触发），所以**停着的机器人订阅它会永远收不到**。要看同一估计请直接读 TF。

==========================================
==========================================

# 第二部分：写本文档之后的进展（abort 循环的真正分解与端到端复测）

> 本部分记录的是本文档（§0–§5）写完之后才发生的工作：三个此前**未被识别**的缺陷被定位并修复，
> 端到端五点巡游从"从未跑完"变成 **5/5 全部成功**，并留下一个已查清但**尚未修**的残余停顿。
> 阅读时请注意：**本部分覆盖前文的若干结论**，覆盖关系集中在 §13。

## 6. 主结论：abort 循环不是一个缺陷，是三个独立缺陷叠加

前文 §2.1 把 abort 循环归因为 AMCL 定位错误（误差 2.28 m / 65°）。**这个归因只对了一半。**
定位问题（§2.2–§2.3）确实存在，但修掉定位之后，失败依然照旧——只是失败签名换了。

| # | 缺陷 | 层级 | 状态 |
|---|---|---|---|
| 1 | 驱动轮轴心不在 `base_link` 原点，原地转向时 `base_link` 画半径 0.15 m 的圆 | 几何 / URDF | **已修，已用数据证明** |
| 2 | 253(INSCRIBED) 硬墙比机器人真实半宽还宽，起点落在 253 格上 → NavFn 直接返回失败 | 规划 / costmap | **已修** |
| 3 | 进度检查器只有平移项，一次 >190° 的原地转向必然超时 abort | 控制 / 参数 | **已修** |
| 4 | `Oscillation` critic 的二值闩锁 + 4 cm/3° 的蠕动永远解不开闩锁 | 控制 / 算法 | **已查清，未修（见 §11）** |

判定"缺陷 1 已修"的分水岭是第三次复测（§10 的 run1）：三条失败腿的**定位误差只有 0.009–0.045 m**，
却依然停在 y ≈ −2.0。也就是说，**定位已经好了，剩下的失败跟定位无关**——这才把注意力推到 §8 的规划级死锁上。

## 7. 缺陷一：驱动轮轴心偏离 `base_link` 原点（已修，已证明）

**现象.** 原地自转时 `base_link` 不绕自身旋转，而是绕驱动轴心画一个半径 **0.150 m** 的圆。
表现为 `/wheel/odom` 真值与 TF 估计在自转时系统性发散，AMCL 的粒子云周期性炸开。

**根因（写在 `robot.urdf.xacro` 里的推导）.** 两个驱动轮 joint 的 `origin x` 都是 `0.15`，
而 `base_link` 在 `x=0`，于是原地转向的瞬时旋转中心（ICR）落在驱动轴上而不是车体原点上。
AMCL 的差速运动模型把 `delta_rot1` 算成 `-(90° - dyaw/2)`，它的噪声项

```
delta_rot1_noise = min(|rot1|, |rot1 - pi|) ≈ pi/2
```

**与步长无关**，于是每帧注入

```
sigma_trans = sqrt(a3*d^2 + 2*a4*(pi/2)^2) ≈ 0.47 m
```

而真实位移只有 0.03 m —— **噪声是信号的 15 倍**，粒子云直接炸开。
注意这条噪声与环境对称性、与地标尺寸都无关，是纯粹的运动学缺陷。

**证据.**

| 量 | 修改前 | 修改后 |
|---|---|---|
| 轴心半径（实测） | **0.150 m** | **0.000 m** |
| 中位 `rot1` | −85.7° | **+0.0°** |
| 中位 `sigma_trans` | 0.465 m | **0.046 m** |

**处置（已应用）.**

| 位置 | 原 | 现 |
|---|---|---|
| `front_left_wheel_joint` / `front_right_wheel_joint` 的 `origin` | `0.15 ±0.2 0.0` | **`0.0 ±0.2 0.0`** |
| 脚轮 | `back_left_caster` / `back_right_caster`，两个都在 `x=-0.15` | `rear_caster` / `front_caster`，中心线上 `x=∓0.20` |
| 脚轮碰撞体 | `cylinder radius=0.04 length=0.04` | `sphere radius=0.05` |
| 新增 Gazebo 摩擦块 | 无 | `mu1=mu2=0.01, kp=1e6, kd=1.0` |

**为什么脚轮必须跟着改（容易漏掉的一环）.** 轴心搬到 `x=0` 之后，两个脚轮如果还都在 `x=-0.15`，
重心正好压在支撑多边形边缘，车会向前栽。所以脚轮改成"前后各一个、走在中心线上"。
中心线上的球体脚轮只负责支撑，`mu=0.01` 近似无摩擦，**不参与转向、不影响 ICR**。

> 顺带说明前文 §2.7 的"脚轮半径比驱动轮小 1 cm"：那个 1.91° 永久俯仰的问题依然真实，
> 但改球体脚轮时一并解决了。**但 §2.7 说的"扫描平面擦到自身结构"不是本次 abort 循环的主因**，
> 主因是本节这个轴心缺陷。

## 8. 缺陷二：253 硬墙比机器人还宽 → 规划级死锁（已修）

**现象.** 失败腿的终点高度一致地停在 **y ≈ −1.97 ~ −2.00**，航向 ≈ ±180°。
前序会话的 Nav2 日志里有 **42 次** `Failed to create a plan`。

**决定性测量（`/tmp/nav-diag/p2.txt`，直接解码 costmap 原始像素）.**

```
=== GLOBAL costmap (decoded) ===
  101x101 @ 0.050  origin (-2.53,-2.54)
  unknown     0 (0.0%)   free  3181 (31.2%)
  LETHAL    983 (9.6%)   INSCRIBED  3008 (29.5%)   inflation(1..98)  3029 (29.7%)
  planner-blocking = LETHAL + INSCRIBED = 3991 cells (39.1%)

=== 失败腿的起点格 (GLOBAL) ===
  4 FAIL start (-1.20,-1.97)  published 99 -> internal  253  INSCRIBED -- NavFn wall
  5 FAIL start (-1.00,-1.99)  published 99 -> internal  253  INSCRIBED -- NavFn wall

=== 机器人当时自认为的位置 (-0.644, -1.984) ===
  global cell (37,11) published 99 -> internal 253 INSCRIBED = NavFn wall
```

**根因.** 两个事实相乘：

1. **NavFn 把 253 也当障碍。** `NavfnPlanner::setCostmap` 里 `>= COST_OBS_ROS(253)` 的格子
   一律按障碍处理，起点格落在 253 上时 `calcNavFnDijkstra` **直接返回 false** ——
   这就是那 42 次 `Failed to create a plan`。
2. **253 硬墙的厚度 = `min(inscribed_radius, inflation_radius)`。**
   原配置 `robot_radius: 0.30`（把车写大了一整圈）配 `inflation_radius` 0.05/0.12：
   膨胀循环**在到达 inscribed 之前就停了**，1..252 的渐变带完全不存在，
   DWB 因此一路贴墙走；而把 inflation 提到 0.35 之后，253 墙变成 0.30 m，
   **比机器人真实的 0.25 m 半宽还宽** —— 墙比车宽，规划器无解，死锁。

**处置（已应用）.**

| 参数 | 原 | 现 |
|---|---|---|
| `local_costmap` / `global_costmap` 的尺寸描述 | `robot_radius: 0.30` | 显式 `footprint: "[[-0.25,-0.25],[-0.25,0.25],[0.25,0.25],[0.25,-0.25]]"` |
| `local_costmap.inflation_layer.inflation_radius` | 0.05 | **0.60** |
| `global_costmap.inflation_layer.inflation_radius` | 0.12 | **0.60** |
| 两者 `cost_scaling_factor` | 3.0 | **3.0（保持不动）** |
| `global_costmap` 插件列表 | 含 `obstacle_layer` | **删除 `obstacle_layer`** |

真实包围盒是 0.5×0.5，真实内切半径 0.25；Nav2 从 footprint 算得 inscribed = 0.25、
circumscribed = 0.354。于是 253 硬墙厚度 = **物理半宽 0.25 m**，外面留 0.35 m 渐变带——
墙和车一样宽，规划器有解，DWB 也有梯度可用。

**删除 global `obstacle_layer` 的理由.** 它把实时 `/scan` 变换到 map 帧后画进全局代价地图，
一旦 AMCL 有漂移，就会**把障碍物画在错误的位置**上，反过来制造幽灵墙和幽灵通路。
（`local_costmap` 的 `obstacle_layer` 保留——局部地图就该用实时传感器。）

**效果.** 最终一轮巡游 `Failed to create a plan` = **0**。

## 9. 缺陷三：进度检查器只看平移（已修）

**现象.** `Failed to make progress` 在整段会话里出现 **34 次**，每次都紧跟着 abort。

**根因.** `nav2_controller::SimpleProgressChecker` 内部的 `pose_distance()` **只有平移项，
没有角度项**。一次 180° 原地转向：0.7 s 加速 + 8.6 s 巡航 ≈ **9.3 s**，
已经贴着 `movement_time_allowance: 10.0` 的窗口；**超过 190° 的转向必然 abort**。

**处置（已应用）.**

```yaml
progress_checker:
  plugin: "nav2_controller::PoseProgressChecker"   # 原 SimpleProgressChecker
  required_movement_radius: 0.10                   # 保留
  required_movement_angle: 0.25                    # 新增（rad，约 14.3°）
  movement_time_allowance: 10.0                    # 保留
```

`PoseProgressChecker` 继承 `SimpleProgressChecker`，额外比较姿态角，官方就是用它把"原地旋转"
算作有进展。已在容器里用 `ros2 param dump /controller_server`（`/tmp/cs.yaml`）确认**线上生效**。

## 10. 端到端复测：四次 2/5 → 最终 5/5

### 10.1 中间四次失败的复测

| 轮次 | 时间 | 结果 | 关键量 |
|---|---|---|---|
| `tour_run1` | 18:55 | 2/5 | goal2 到达距离 **0.315 m > 0.30 容差**；goal3/4/5 ABORT 于 (−1.20,−1.97,+168.6)/(−1.00,−1.99,+177.4)/(−0.85,−2.00,+179.4)，**定位误差仅 0.009 / 0.013 / 0.045 m** |
| `tour_run2` | 19:11 | 2/5 | goal2 ABORT 75.1 s，**定位误差 1.399 m / −57.55°**（AMCL 镜像锁定）；脚本被 nav 重启打断（`ExternalShutdownException`） |
| `tour_fix1` | 19:18 | 2/5 | goal2 用 142.8 s 成功但停顿 12.7 s；goal3/4/5 ABORT，定位误差 0.358 / 0.251 / 0.105 m |
| `tour_fix2` | 19:30 | 2/5 | goal2/4 **TIMEOUT 150 s**，定位误差 **1.609 / 2.562 m**（又是镜像锁定） |

**run1 是分水岭。** 定位误差 0.009–0.045 m 却依旧失败，说明**失败已经不是定位问题**。
这也解释了为什么 run2/fix2 会退回去：那两轮跑在 §8 的 footprint/inflation 改动**之前**，
镜像锁定仍在污染全局地图。注意 run2/fix2 的镜像锁定误差（1.4–2.6 m）与前文的 2.28 m 同源，
属于 §2.2 那个尚未修的多峰根因，**它在本轮是靠 §8 删掉 global `obstacle_layer` 缓解的**。

### 10.2 最终一轮：5/5 全部成功

```
5 goals, timeout 180 s each
--- goal 1/5  (+1.60, +1.60, +0 deg)  ->  SUCCEEDED  105.5 s   终点距离 0.195 m  定位误差 0.072 m, +0.74°   (停顿 10.0 s)
--- goal 2/5  (-1.60, -1.60, +0 deg)  ->  SUCCEEDED  139.3 s   终点距离 0.204 m  定位误差 0.025 m, -2.01°   (停顿 11.2 s)
--- goal 3/5  (-1.60, +1.60, +0 deg)  ->  SUCCEEDED   76.5 s   终点距离 0.200 m  定位误差 0.043 m, -0.91°   (停顿 10.1 s)
--- goal 4/5  (+1.60, -1.60, +0 deg)  ->  SUCCEEDED   33.2 s   终点距离 0.229 m  定位误差 0.063 m, -0.76°
--- goal 5/5  (+0.00, +0.00, +0 deg)  ->  SUCCEEDED   31.2 s   终点距离 0.190 m  定位误差 0.061 m, +1.06°   (停顿 10.0 s)
5/5 goals SUCCEEDED
trajectory: /tmp/nav-diag/tour_traj.csv (88243 samples)
```

- 到达距离 0.190–0.229 m，全部在 0.30 m 容差内（几乎没有富余，见 §11 的说明）；
- 到达时的定位误差 0.025–0.072 m —— 与 §2.1 的 2.28 m 相比是**两个数量级**的改善；
- 绕行比 1.03–1.19×，总耗时 385.7 s。

**Nav2 日志签名（整段会话）.**

| 签名 | 次数 |
|---|---|
| `Goal failed` | **0** |
| `Failed to create a plan` | **0** |
| `Collision Ahead` | **0** |
| `Failed to make progress` | 34 |
| `Aborting handle` | 34 |
| `Running backup` / `spin` / `wait` | 各 4 |

**关键读法：34 次 abort 但 0 次目标失败。** 系统是自愈的——每次停顿最终都被恢复行为打破，
代价是时间。这就是 §11。

## 11. 残余停顿：`Oscillation` 闩锁的完整因果链（已查清，**未修**）

五次成功的巡游里，三条腿中途出现 10–12 s 的停顿。它不致命，但每条受影响的腿要多花 40–100 s。

**完整链条（每一步都有数据）：**

1. 控制指令链在停顿段是**忠实的**：DWB 确实选了 `v = 0.000 / +0.008`，
   而同时 **378 条合法的"行驶"轨迹**就摆在那里。（`chain.py`，`chain.log` 阶段一 t=0–19.6 s）
2. **决策裕度只有约 1%**：获胜轨迹的总分 69.00，与它几乎并列的也是 69.00；
   其中 `GoalAlign + GoalDist` 贡献了约 69 里的 **59.4（约 86%）**，而这两项的 raw 只在 **49–58** 之间张望。
3. **`Oscillation` 是二值闩锁**：raw 只取 `+0.000` 或 `−1.000`，419 条轨迹里 **200 条是 −1.000**。
   本工程的 `oscillation_reset_dist: 0.05`（5 cm）、`oscillation_reset_angle: 0.2`（~11.5°）、
   `oscillation_reset_time: -1.0`（禁用）意味着**一次 4 cm / 3° 的蠕动永远解不开闩锁**；
   它在交替的周期里否决 378 条行驶轨迹中的 **180 条**。
4. 10 s 进度检查器到点 → `Failed to make progress` → `Aborting handle`。
5. 行为树跑 `BackUp`，以 0.05 m/s **倒车 0.295 m**。
6. **0.295 m ≫ `oscillation_reset_dist: 0.05`** → 闩锁清除。
7. 控制器恢复满速（`v=+0.103…+0.150, w=+0.350`），该腿完成。

**`BackUp` 绕过 velocity_smoother（一个容易误判的点）.**

```bash
$ ros2 topic info /cmd_vel -v
  publisher: behavior_server   (spin, backup, drive_on_heading, assisted_teleop, wait)  ← 5 个
  publisher: velocity_smoother
  subscriber: diff_drive       ← 唯一
```

恢复行为**直接发到 `/cmd_vel`**，不经过 smoother。这是 Humble 的原生布局（不是本工程的配置错误），
但它解释了 §12.3 里"`nav_v != cmd_v`"那一段。

**净效果.** 系统自愈：**34 次 abort、12 次恢复行为，0 次目标失败**；
每次停顿约 20–30 s，受影响的腿多花 40–100 s。

**建议的修法（⚠️ 未应用、待用户批准）.**

```yaml
progress_checker:
  movement_time_allowance: 10.0 -> 4.0
```

理由：真正解开死锁的是 **abort → 恢复行为** 这个循环本身，所以让它**更早触发**严格更快。
安全性：0.15 m/s × 4 s = 0.6 m，远大于 `required_movement_radius: 0.10`，正常行驶不可能误触发。

> **我没有动 `oscillation_reset_time`**，因为它在当前 nav2 版本里的确切语义我没有验证过。
> 若要根治而非缓解，应该从这个闩锁本身入手（提高 `oscillation_reset_dist` 或缩短 `reset_time`），
> 但那需要先做验证实验。

**一个尚未解释的观察（留给后续）.** 有一轮在"真值距目标 0.181 m、yaw 差 0.267 rad，
两者都在容差内"的情况下**没有宣布成功**，重启之后才成功。需要查那一刻 AMCL 的置信位姿——
怀疑仍是 §2.2 的多峰在近端造成的 belief/truth 分歧。

## 12. 新增的错误假设与工具坑（继续记，避免后人重犯）

### 12.1 `CriticScore.scale` 不是配置里的 scale，是被归一化过的

配置值 `GoalAlign 24.0` 线上报 **0.6**；`PathAlign` / `PathDist 32.0` 报 **0.8**；
`GoalDist 24.0` 报 **0.6**（除数 40）。`RotateToGoal` 保持 32.0、`Oscillation` 保持 1.0。
**并且：作出否决的 critic 报 `scale = 0.0`。**
推论：`Σ(raw × scale)` 的绝对值没有跨消息的意义，**只能在同一条 `/evaluation` 消息内部比较**。

### 12.2 DWB 把负 `raw_score` 直接当成"轨迹非法"，与 scale 无关

所以 `Σ(raw × scale)` 只要有一项 raw 为负就**不是有效代价**。排序前必须先按
`all(c.raw_score >= 0)` 过滤，否则被否决的轨迹会排在最前面。

### 12.3 自我更正：我曾断言"`nav_v == cmd_v` 几乎每一行，velocity_smoother 什么都没吃"

**这是过度概括，已更正。** 停顿段（t=0–19.6 s）成立；**恢复段（t=20.7–27.4 s）不成立**——
那一段 `nav` 是过期的 0，而 `/cmd_vel` 上是 −0.050，真值从 x=−1.65 走到 −1.94。
机械原因就是 §11 的"恢复行为绕过 velocity_smoother"。

### 12.4 被证伪的假设

| 假设 | 证伪依据 |
|---|---|
| velocity_smoother 在破坏 DWB 的指令 | 对本控制器的指令它是忠实的 passthrough（`feedback: OPEN_LOOP`、`scale_velocities: False`） |
| 碰撞 / 代价地图是停顿原因 | `BaseObstacle` 对所有获胜或停住的轨迹 raw 都是 `+0.000`；机器人自己所在格代价 0 |
| 需要重启 Nav2 才能拿到 DWB 调试输出 | 不需要，`/evaluation` 一直在发（`publish_evaluation: true`） |

### 12.5 `short_circuit_trajectory_evaluation: true` 的用法

它会把非法轨迹的 `scores` **截断在否决它的那个 critic 上**，
所以 **`scores` 的最后一条的名字就是"凶手"**。这是一个可以直接读出来的诊断口。

### 12.6 `behavior_server` 的 `backup_*` 参数根本没有被设置

```bash
$ ros2 param get /behavior_server backup_speed   # -> Parameter not set
```

`BackUp` 跑的是 nav2 默认值（实测 **0.050 m/s**；我原先记的 0.025 m/s 是错的）。
`global_frame` 返回 `odom`。

### 12.7 诊断脚本自身的坑（新发现）

| 脚本 | 缺陷 | 状态 |
|---|---|---|
| `evalwatch.py` | `ranked = sorted(scored, ...)` **没有按合法性过滤**；被 `Oscillation` 否决的轨迹截断后 total = 0.000，会排在所有合法轨迹（~68–69）前面 | 已诊断，**未修**；修法已在 `chain.py` 里（`all(c.raw_score >= 0 ...)`），尚未回移 |
| `goal_seq.py` | 停顿检测**只看平移**，原地转向被误报成 `!! STALLED` | 未修 |
| `chain.py` | `nav` / `cv` / `ev` 没有时间戳，过期值会当成实时值打印 | 未修 |
| `stallprobe.py` | 最近格打印的实参顺序写反；行循环永远不打印机器人自己那一行 | 未修 |

### 12.8 环境坑（补充 §5）

```bash
# 裸 docker exec 下 ros2 node list 什么都不返回，必须先 source（source 后列出 30 个节点）
docker exec humble_nav2 bash -lc 'source /opt/ros/humble/setup.bash && \
  source /root/ros/ros2humble_nav_ws/install/setup.bash && ros2 node list'

# 容器时钟 ≈ UTC，主机 ≈ UTC+8，比对日志时间戳时要减 8 小时
# 主机 python 有 numpy 2.5.3，但没有 matplotlib —— 轨迹绘图需要另想办法
```

## 13. 状态更新：本部分覆盖了前文的哪些结论

| 前文 | 现状 |
|---|---|
| §4.3-1「**端到端复测从未跑完**」 | **已被 §10.2 的 5/5 五点巡游取代** |
| §2.6 / §4.2 的 `inflation_radius` 0.05→**0.35**、`cost_scaling_factor` 3.0→**1.5** | **已被覆盖**：现在是显式 `footprint`（内切 0.25）+ `inflation_radius: 0.60` + `cost_scaling_factor: 3.0` |
| §2.1「abort 循环 = AMCL 定位错 2.28 m」 | 收敛为**三个独立缺陷**（§7/§8/§9）**加一个残余停顿**（§11）；定位只是其中之一 |
| §2.7「脚轮半径不一致 → 永久俯仰」 | 依然真实，但**不是** abort 循环主因；改球体脚轮时一并解决 |
| §2.8「AMCL 没有里程计订阅」 | 结论不变，仍然成立 |

**工作区当前 `nav2_params.yaml` 相对 HEAD 的全部改动**（供裁决用）：

| 参数 | HEAD | 当前 | 是否经用户批准 |
|---|---|---|---|
| `amcl.max_beams` | 60 | 120 | **否**（前序会话遗留） |
| `amcl.recovery_alpha_fast` / `_slow` | 0.0 / 0.0 | 0.1 / 0.001 | **否**（前序会话遗留） |
| `controller_server.odom_topic` | 无（默认 `odom`） | `/odometry/filtered` | 是（§2.4） |
| `velocity_smoother.odom_topic` | `odom` | `/odometry/filtered` | 是（§2.4） |
| `progress_checker.plugin` | `SimpleProgressChecker` | `PoseProgressChecker` + `required_movement_angle: 0.25` | 是（§9） |
| 两个 costmap 的尺寸描述 | `robot_radius: 0.30` | 显式 `footprint`（内切 0.25） | 是（§8） |
| 两个 costmap 的 `inflation_radius` | 0.05 / 0.12 | 0.60 / 0.60 | 是（§8） |
| `global_costmap` 的 `obstacle_layer` | 启用 | 从插件列表移除 | 是（§8） |
| `progress_checker.movement_time_allowance` | 10.0 | 10.0 | —— **建议改 4.0，尚未应用**（§11） |

**仍未完成的项.**

1. §11 的 `movement_time_allowance: 10.0 → 4.0` —— **待批准**。
2. §2.2 的多峰根因**依然未修**，只是靠删掉 global `obstacle_layer` 把它的后果压下去了；
   `tour_run2` / `tour_fix2` 的镜像锁定（1.4–2.6 m）就是它。
3. **停顿瞬间的 `/plan` 从未被采集过**（`stallprobe2.py` 写好了但没跑）——
   这是最后一个未闭合的环节，用来区分"DWB 的代价函数坏了"和"规划器交给 DWB 的路径本来就到不了"。
4. §12.7 的四个脚本缺陷。
5. 轨迹绘图：主机没有 matplotlib，需要容器侧装、或用纯 numpy 输出 SVG、或做成 HTML。

## 14. 本部分新增的诊断脚本

| 脚本 | 用途 |
|---|---|
| `chain.py` | 5 Hz 采样整条控制链（`nav` / `cmd` / `odom`），**只对合法轨迹排序**，是 §11 的直接证据来源 |
| `evalwatch.py` | 停顿瞬间读 DWB 自己的打分（`/evaluation`）：行驶轨迹数、被谁否决、获胜轨迹的逐 critic 分解 |
| `stallprobe2.py` | 在停顿瞬间抓 `/plan`（机器人坐标系）+ 最近的行为树状态迁移 —— **尚未运行** |
| `goal_seq.py` | 五点巡游端到端测试（**本次跑完全程：5/5**），输出 `tour_traj.csv` |
| `p2.txt` | 全局 costmap 的原始像素解码（§8 的 253 硬墙证据） |


## 两处需要你裁决/知情

1. §11 的 movement_time_allowance: 10.0 → 4.0 我没动，文档里是"待批准"。批准后我改完按之前的约定重跑五点巡游。
2. §4.3-1「端到端复测从未跑完」现在已在文档里被明确标注为"已被取代"，但为了保持附录只增不改，§5 脚本表里 goal_seq.py 那行"被 kill，未完成"的原文保留着——覆盖关系写在 §13。如果你希望我直接改 §5 那一行，说一声。
==========================================
==========================================

# ch03.

修好了，全部在容器里的实机（活仿真）上验证过。

病根:
nav2_params.yaml 里 amcl.set_initial_pose: True + initial_pose: (0,0,0)，前提是「nav2 启动那一刻机器人在出生点」。仿真常开、nav2 反复重启时这个前提不成立；而 AMCL 只在机器人移动时才做激光更新（update_min_d/a），所以停着的机器人会一直挂着错位姿，还会越漂越远——实测放着不管从 1.13 m 漂到 3.42 m。

修法：新增常驻节点 scripts/amcl_seed_pose.py

挂在 simulation.launch.py 里（跟仿真同生命期），nav2 的启动命令不用改：

1. 订阅 /amcl/get_state，等 AMCL 进 ACTIVE 才动手。必须等：amcl_node.cpp::on_activate() 里只要 set_initial_pose=True，它必然先用参数里的 (0,0,0) 走一遍，激活之前发的 /initialpose 一定被盖掉。参数里那对 (0,0,0) 留着当引子（没它 map→odom 出不来）。
2. 先进 PROBE：/request_nomotion_update 逼一帧 /amcl_pose（AMCL 停着不发）。差 ≤ 10 cm 就不动它（激光估计比里程计信息量大，没必要拿真值盖），否则按 /wheel/odom 真值播 3 帧 /initialpose。
3. 进 VERIFY：再逼一帧，差 ≤ 25 cm 收工；超时最多重播 3 轮，再不行打一条明确警告让人手动 2D Pose Estimate。
4. 进 OFF 待命；AMCL 一旦离开 ACTIVE（重启/被杀），下次激活重走一遍——所以重启几次就重新播几次。

改动文件：新增 scripts/amcl_seed_pose.py；CMakeLists.txt（install(PROGRAMS)）、package.xml（exec_depend）、launch/simulation.launch.py（挂节点）、config/nav2_params.yaml（注释改写成现在这套机制）。已 colcon build --symlink-install --packages-select robot_nav_bringup。

验证结果

播种链路（故意把 AMCL 信念推歪 0.8 m 后跑）：

[1/3] AMCL 认为 (+2.556, +2.225, +35.8 deg)，与真值 (+0.097, -0.152, +35.9 deg) 差 3.421 m，播种
   [amcl] Setting pose (5284.800000): 0.097 -0.152 0.626      ← 播的就是真值，×3 条
播种生效：/amcl_pose 与真值差 0.064 m，已对齐
read     AMCL 认为 (+0.098, -0.223, +36.5 deg) | 真值 (+0.097, -0.152, +36.1 deg) | 差 0.072 m   ← 独立复核

对照组（同样推歪、什么都不做）：AMCL 不自我恢复，反而漂到 3.418 m。重新武装这条路（也就是你重启 nav2 的场景）用一个假 lifecycle 节点单独验过：ACTIVE → 探测；转 INACTIVE → AMCL 不在激活态了，等它下次激活再播种；再回 ACTIVE → 重新探测（差 0.063 m，不动它）；把tate 问不到） 并重新武装。SIGTERM 下干净退出，无 traceback。

两条踩过的 AMCL 内部事实（已写进代码 docstring，省得下次再查）：

- Failed to transform initial pose in time 这行 WARN 是 AMCL 自己打的，无害：handl是 now() 而不是消息 stamp，catch 里只打日志然后按恒等 odom增量继续——位姿照样生效，紧接着那句 Setting pose (…): x y yaw 就是证据。四种 stamp（odom 戳 / 节点时钟 / odom−0.1 s / 零戳）全都 5/5 报它，从发送端无解，别追。播种收敛到 3 发（v1 的 5 Hz×6 s
  每次 nav2 启动能刷 30 条）。
- 干净启动（机器人真在出生点）时误差约 0.07 m < 10 cm，节点什么都不做，不会干扰正常流程。

需要你亲手跑

我不动你的进程（sim pid 64136/64139、nav2 69975 还都在跑）。要生效得重开仿真（新节点在 simulation.launch.py 里）+ 重开 nav2，命令都不变：
```sh
# 容器内，仿真
ros2 launch robot_nav_bringup simulation.launch.py extra_gazebo_args:=-slibgazebo_

# 容器内，另一个终端，nav2（原样）
ros2 launch nav2_bringup bringup_launch.py map:=/root/ros/ros2humble_nav_ws/maps/demo_map.yaml \
  use_sim_time:=true params_file:=/root/ros/ros2humble_nav_ws/src/robot_nav_bringu
```
起来后验收看两行就够：AMCL 激活时应该出现 AMCL 激活了，先问一帧 /amcl_pose 对答案 x m），不动它，要么 [1/3] … 播种 → 播种生效：… 已对齐。想单独复核，另开一个终端跑ros2 run robot_nav_bringup amcl_seed_pose.py，它只会读、不会乱播。

顺带要报的两件事（这次没动）

1. 5 目标 tour 5/5 成功（15.6 / 43.2 / 34.1 / 33.3 / 67.4 s，终点误差 0.191–0.251 m，容差 0.30）——定位喂对之后，对比修之前的 2/5 基线。
2. 第 5 个目标暴露了另一个独立缺陷：DWB 近零指令死锁，跟 AMCL 无关（全程定位误差 ≤4) 真冻住 44 s，vx 恰好 0.000，每 10 s 净位移 ≤ 0.014 m、转角 ≤33°，两个方向都够不到 PoseProgressChecker 的门槛（0.1 m / 0.25 rad / 10 s），于是合规地 abort 了 4 次（间隔 ~10.2 s），最后靠 BT 恢复动作的原地旋转（wz 1.0）才解冻。可选项：(a) 就这么留着，它
   44 s 内自愈；(b) 调 DWB 让它别掉进近零指令态（vx_samples / min_vel_x、RotateToG 介入。要动哪个你说，我不擅自改参数。
3. /gazebo/model_states 和 /get_entity_state 在 graph 里缺失（当初所以改用 /wheel/odom 当真值）。低优先级，现在没影响。

杂项：清掉了我自己留下的后台残留（/tmp/rosout_rec.py 记录器、几个 tail 监控、约 30 个临时日志），容器 /tmp 里还剩 ~110 个我这次调试用的探针脚本（probe*.py、diag*.py、lm_*.py 之类）和 /tmp/setpose.py，要清就说一声——setpose.py 我默认留着。/tmp/nav2.log、/tmp/sim.log 的，没动。