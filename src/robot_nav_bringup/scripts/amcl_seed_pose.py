#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AMCL 播种位姿：AMCL 每次激活时用里程计真值替掉写死的 initial_pose；
之后一直盯着，信念一旦跑偏（尤其是被一発手动 /initialpose 带歪）就再播一次。

问题
----
nav2_params.yaml 里 amcl.set_initial_pose=True + initial_pose=(0,0,0)，这个前提只在
"nav2 启动那一刻机器人正好停在出生点"时成立。实际用法是仿真一直开着、nav2 反复重启：
nav2 起来时机器人早已被上一轮测试开走，AMCL 的粒子云却被按在出生点。而 AMCL 只在
机器人移动时才做激光更新（update_min_d / update_min_a），所以机器人不动时这个错位姿
会一直挂着：global_costmap 照错位姿绘制，规划出来的路径和真实环境对不上，progress
checker 到点就报 "Failed to make progress" -> abort -> 清代价地图 -> 重试。

    2026-10-14 实测：真值 (0.226, -1.353, 173.0°)，AMCL 认为 (0.001, 0.001, 0.125°)，
    差 1.37 m / 173°。手工发一次 /initialpose 后收敛到 0.025 m。

做法
----
本节点跟着仿真常驻，用 lifecycle 的 get_state 盯着 AMCL：它一进入 ACTIVE 就先逼一帧
/amcl_pose 对答案，和真值差得远才播种（差得近就不动它——激光估计本来比里程计更有信息
量，没必要拿真值去覆盖）。播完继续留在 WATCH 态常看，跑偏了再播。**不需要改动 nav2 的
启动命令**。

为什么还要盯"朝向"（2026-10-14 第二次实测的教训）
----------------------------------------------
第一版只比 (x, y) 距离，这一版把 yaw 也拉进来、并且不再"探一次就收工"。翻车现场是：
14:04:12.73 有一发 /initialpose 把位姿设成 (0.024, -0.025, 172.9°)——位置分毫不差就是
odom 真值，朝向却和真值差 180°（出生点这一带的地图还残留绕世界原点的对称性）。
第一版探到"差 0.003 m"就认定对上了，朝向那 180° 根本没看；之后机器人一动，AMCL 的
粒子云整体落在镜像盆地里，belief 与真值刚性相差 178°，controller 一路
"Failed to make progress" -> Goal failed。
用地图做光线投射量过：出生点处"真值位姿"和"绕原点镜像位姿"打出来的雷达，现在只有
38% 的束相符（改图之前是 80% 相符）——图是能分辨了，但 AMCL 只会做局部更新（没有全局
重定位），一旦整团粒子都落错盆地，它自己爬不回来（实测 90+ s 没回来）。
结论：错位姿只能从入口上堵，所以本节点在 ACTIVE 期间常驻比对 xy+yaw。

为什么是"激活之后"而不是"启动时"
--------------------------------
amcl_node.cpp 的 on_activate()：只要 set_initial_pose=True，它就必然用参数里的
(0,0,0) 走一遍 initialPoseReceived()，而且此时 active_ 已经置位，所以**激活前**发过去
的 /initialpose 一定会被这发 (0,0,0) 盖掉（那条 init_pose_received_on_inactive 分支
根本轮不到）。等 get_state 报 ACTIVE 再播，就落在它的后面，一发就定。参数里的
(0,0,0) 留着当引子：AMCL 激活时需要有个初始位姿，否则 map->odom 出不来、
global_costmap 起不来。

关于日志里的 "Failed to transform initial pose in time"
-----------------------------------------------------
这是 amcl_node.cpp::handleInitialPose() 自己打的，跟本节点怎么填时间戳无关：
它要按 (base_link@消息时间 -> base_link@now)，以 odom 为固定系查一次"这段里程计变化"
（就是把"过期的位姿估计"补偿到当前），其中 now() 取的是仿真时间。仿真的 /clock 比
TF 里 odom->base_link 的最新样本快 3ms 左右，于是这发查询必然 extrapolation。实测把
时间戳改成 odom 戳-0.1s、甚至零戳（tf2 的 latest 语义），警告照样出——因为失败的时间
点就是 now()。看代码，catch 里只是打日志然后用单位变换继续（"ignoring this startup
condition doesn't really cost us anything"），位姿照常 apply，
后面紧跟的 "Setting pose (…)" 就是证据。所以：无害，且不是本节点引入的（rviz 手点
2D Pose Estimate 同样会出）。本节点把播种收敛到 1~3 发，就是为了别让这行日志刷屏。

为什么 /wheel/odom 可以直接当真值
--------------------------------
该 diff_drive 插件的 odometry_source 是 world（不是编码器积分），实测其位姿与
`gz model -m nav_robot -i` 逐位吻合（<1 mm），而且 odom 系原点就是世界原点。所以
"odom 里的位姿" == "真实世界位姿" == "map 里的位姿"（demo_map 就是照这个世界建的，
origin=[-2.53,-2.54] + 99x99@0.05 覆盖 ±2.47m）。三者相等是本工程成立的前提，也正是
原来那句 initial_pose=(0,0,0) 的依据。
也正因为如此，拿真值去纠 AMCL 是安全的；真机（轮子会打滑）上就得把 watch_enabled
关掉或放宽阈值。

参数
----
  odom_topic      真值来源，默认 /wheel/odom
  pose_frame      播种时填的 frame_id，默认 map（AMCL 按 global_frame_id 校验它）
  amcl_node       AMCL 节点名，默认 /amcl，用来拼它的 get_state 服务名。
                  （/initialpose、/amcl_pose、/request_nomotion_update 都用相对名，
                   和 AMCL 在根命名空间下建的名字自然一致）
  xy_var/yaw_var  播种协方差，默认 0.05 m^2 / 0.02 rad^2
  seed_burst      每次播种连发几条，默认 3（防丢包，够用又不刷屏）
  seed_interval   连发间隔，默认 0.2 s
  skip_tol        激活后探到 xy 比这个还准就不播了，默认 0.10 m
  yaw_tol         朝向容差：探到 / 盯到 偏差超过它就认为跑偏，默认 0.35 rad ≈ 20°
  verify_tol      播完确认 xy 对齐的容差，默认 0.25 m（激光更新后信念有几厘米抖动）
  verify_timeout  播完等对齐的上限，默认 6.0 s
  probe_timeout   激活后等 /amcl_pose 的上限，默认 5.0 s；等不到就直接播
  max_attempts    一次激活（或一轮看门狗）最多播几轮，默认 3
  watch_enabled   激活后是否常驻盯着，默认 True
  watch_xy_tol    盯梢时允许的 xy 偏差，默认 0.35 m
  watch_interval  盯梢间隔，默认 1.0 s
  watch_persist   偏差要连续保持这么久才动手，默认 2.0 s（滤掉单帧毛刺）
  watch_stale     信念超过这么久没更新就逼一帧 nomotion update，默认 3.0 s
  watch_backoff   一轮播种最终没对齐后，隔这么久再让看门狗动手，默认 20.0 s
  odom_buffer     缓存多少秒的 odom 用来对齐时刻，默认 5.0 s
"""

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty

WAIT, PROBE, SEED, VERIFY, WATCH = 0, 1, 2, 3, 4

STATE_NAMES = {
    State.PRIMARY_STATE_UNKNOWN: 'UNKNOWN',
    State.PRIMARY_STATE_UNCONFIGURED: 'UNCONFIGURED',
    State.PRIMARY_STATE_INACTIVE: 'INACTIVE',
    State.PRIMARY_STATE_ACTIVE: 'ACTIVE',
    State.PRIMARY_STATE_FINALIZED: 'FINALIZED',
}

POLL = 0.5          # get_state 询问间隔（秒）
POLL_TIMEOUT = 2.0  # 单次询问等回应的上限，超了就当问不到
STATE_TTL = 3.0     # 拿到的状态的有效期，过期视作 AMCL 不在了


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    """a-b 归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class AmclSeedPose(Node):
    def __init__(self):
        super().__init__('amcl_seed_pose')

        self.odom_topic = self.declare_parameter('odom_topic', '/wheel/odom').value
        self.pose_frame = self.declare_parameter('pose_frame', 'map').value
        self.amcl_node = self.declare_parameter('amcl_node', '/amcl').value
        self.xy_var = self.declare_parameter('xy_var', 0.05).value
        self.yaw_var = self.declare_parameter('yaw_var', 0.02).value
        self.seed_burst = self.declare_parameter('seed_burst', 3).value
        self.seed_interval = self.declare_parameter('seed_interval', 0.2).value
        self.skip_tol = self.declare_parameter('skip_tol', 0.10).value
        self.yaw_tol = self.declare_parameter('yaw_tol', 0.35).value
        self.verify_tol = self.declare_parameter('verify_tol', 0.25).value
        self.verify_timeout = self.declare_parameter('verify_timeout', 6.0).value
        self.probe_timeout = self.declare_parameter('probe_timeout', 5.0).value
        self.max_attempts = self.declare_parameter('max_attempts', 3).value
        self.watch_enabled = self.declare_parameter('watch_enabled', True).value
        self.watch_xy_tol = self.declare_parameter('watch_xy_tol', 0.35).value
        self.watch_interval = self.declare_parameter('watch_interval', 1.0).value
        self.watch_persist = self.declare_parameter('watch_persist', 2.0).value
        self.watch_stale = self.declare_parameter('watch_stale', 3.0).value
        self.watch_backoff = self.declare_parameter('watch_backoff', 20.0).value
        self.odom_buffer = self.declare_parameter('odom_buffer', 5.0).value

        self.odom = None
        self.odom_hist = deque()   # (仿真时刻, x, y, yaw)，用来按时刻对齐真值
        self.amcl = None
        self.amcl_t = 0.0          # 收到 /amcl_pose 的时刻（monotonic）
        self.ready_logged = False

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self.on_amcl, 20)
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         'initialpose', 10)
        self.nomotion = self.create_client(Empty, 'request_nomotion_update')

        self.lc = self.create_client(GetState, self.amcl_node + '/get_state')
        self.lc_future = None
        self.lc_sent = -1e9
        self.lc_state = None
        self.lc_t = 0.0
        self.reported_state = -1

    # ---- 回调 ----------------------------------------------------------
    def on_odom(self, m):
        self.odom = m
        p = m.pose.pose
        t = stamp_s(m.header.stamp)
        self.odom_hist.append((t, p.position.x, p.position.y,
                               yaw_of(p.orientation)))
        while len(self.odom_hist) > 8 and t - self.odom_hist[0][0] > self.odom_buffer:
            self.odom_hist.popleft()
        while len(self.odom_hist) > 1000:      # 时间戳全为零时的兜底
            self.odom_hist.popleft()

    def on_amcl(self, m):
        self.amcl = m
        self.amcl_t = time.monotonic()

    # ---- 取值 ----------------------------------------------------------
    def truth(self):
        p = self.odom.pose.pose
        return (p.position.x, p.position.y, yaw_of(p.orientation))

    def truth_at(self, t):
        """仿真时刻 t 的里程计真值；没有可用时间戳就退回最新一帧。"""
        if not self.odom_hist:
            return None
        if not t:
            return self.odom_hist[-1][1:]
        return min(self.odom_hist, key=lambda s: abs(s[0] - t))[1:]

    def belief(self):
        if self.amcl is None:
            return None
        p = self.amcl.pose.pose
        return (p.position.x, p.position.y, yaw_of(p.orientation))

    def loc_err(self):
        """信念与"同一时刻"真值的偏差 (距离 m, 朝向 rad)；缺数据返回 None。

        比的是同一时刻的两边：/amcl_pose 带的是它那帧激光的时刻，就拿那一时刻的
        odom 来比。机器人走着的时候两边时刻对得上，不会因为更新滞后而假报警。
        """
        b = self.belief()
        if b is None:
            return None
        t = self.truth_at(stamp_s(self.amcl.header.stamp))
        if t is None:
            return None
        return (math.hypot(b[0] - t[0], b[1] - t[1]), ang_diff(b[2], t[2]))

    @staticmethod
    def fmt(p):
        return '(%+.3f, %+.3f, %+.1f deg)' % (p[0], p[1], math.degrees(p[2]))

    @staticmethod
    def fmt_err(e):
        return '%.3f m / %+.1f deg' % (e[0], math.degrees(e[1]))

    def off_pose(self, e, xy_tol):
        """xy 或朝向任一项超容差就算跑偏。"""
        return e[0] > xy_tol or abs(e[1]) > self.yaw_tol

    def publish_pose(self):
        """把当前 /wheel/odom 的位姿原样发到 /initialpose（等价于 rviz 的 2D Pose Estimate）。

        时间戳用 odom 自己的（仿真时间），这样不依赖本节点是否已经收到 /clock；
        AMCL 会用 (这个时刻 -> now) 之间的里程计变化把它补偿到当前，所以戳旧一点没关系
        （机器人停着时这段变化本来就是零）。frame_id 用 map，因为 AMCL 要求消息在
        global_frame_id 里。协方差只有 0/7/35 三个下标被 handleInitialPose 读取。
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.pose_frame
        msg.header.stamp = self.odom.header.stamp
        msg.pose.pose = self.odom.pose.pose
        cov = [0.0] * 36
        cov[0] = cov[7] = self.xy_var
        cov[35] = self.yaw_var
        msg.pose.covariance = cov
        self.pub.publish(msg)

    def ask_nomotion(self, asked_flag):
        """逼 AMCL 立刻用当前激光出一帧 /amcl_pose（它平时只在移动时才发）。

        返回更新后的"已请求过"标志：服务没起来就一直算没请求过。
        """
        if not asked_flag and self.nomotion.service_is_ready():
            self.nomotion.call_async(Empty.Request())
            return True
        return asked_flag

    def amcl_state(self, now):
        """AMCL 的 lifecycle 主状态 id；问不到（节点不在/没回应/太久没更新）返回 None。"""
        if self.lc_future is not None:
            if self.lc_future.done():
                try:
                    self.lc_state = self.lc_future.result().current_state.id
                    self.lc_t = now
                except Exception:            # 服务端中途消失
                    self.lc_state, self.lc_t = None, 0.0
                self.lc_future = None
            elif now - self.lc_sent > POLL_TIMEOUT:
                self.lc_future = None
                self.lc_state, self.lc_t = None, 0.0

        if self.lc_future is None and now - self.lc_sent >= POLL:
            self.lc_sent = now
            if self.lc.service_is_ready():
                self.lc_future = self.lc.call_async(GetState.Request())

        if self.lc_state is not None and now - self.lc_t <= STATE_TTL:
            return self.lc_state
        return None

    def note_state(self, state):
        """状态变了才打一行，别刷屏；顺便当 AMCL 的上下线日记。"""
        if state == self.reported_state:
            return
        if state is not None:
            self.get_logger().info(
                'AMCL lifecycle: %s' % STATE_NAMES.get(state, state))
        elif self.reported_state not in (-1, None):
            self.get_logger().info('AMCL 已消失（get_state 问不到）')
        self.reported_state = state

    # ---- 状态机 --------------------------------------------------------
    def run(self):
        st = WAIT
        sent = 0                   # 本轮已发几条
        last_pub = -1e9
        attempts = 0
        t_phase = 0.0
        asked_nomotion = False
        last_watch = -1e9
        viol_since = None          # 看门狗：偏差是从什么时候开始持续的
        watch_ready = 0.0          # 早于这个时刻不让看门狗动手（放弃后的冷却）

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            state = self.amcl_state(now)
            self.note_state(state)

            if self.odom is None:
                if not self.ready_logged:
                    self.ready_logged = True
                    self.get_logger().info('等 %s 出现……' % self.odom_topic)
                continue

            if st == WAIT:
                if state == State.PRIMARY_STATE_ACTIVE:
                    t_phase = now
                    asked_nomotion = False
                    self.get_logger().info('AMCL 激活了，先问一帧 /amcl_pose 对答案')
                    st = PROBE

            elif st == PROBE:
                # 先看它自己认为在哪：已经够准就别用真值去盖掉激光估出来的位姿
                asked_nomotion = self.ask_nomotion(asked_nomotion)
                e = self.loc_err()
                if self.amcl_t > t_phase and e is not None:
                    if not self.off_pose(e, self.skip_tol):
                        self.get_logger().info(
                            'AMCL 位姿已经对上了（差 %s），不动它' % self.fmt_err(e))
                        attempts = 0
                        viol_since, last_watch = None, now
                        st = WATCH
                    else:
                        attempts += 1
                        sent = 0
                        last_pub = -1e9
                        self.get_logger().info(
                            '[%d/%d] AMCL 认为 %s，与真值 %s 差 %s，播种'
                            % (attempts, self.max_attempts, self.fmt(self.belief()),
                               self.fmt(self.truth()), self.fmt_err(e)))
                        st = SEED
                elif now - t_phase >= self.probe_timeout:
                    attempts += 1
                    sent = 0
                    last_pub = -1e9
                    self.get_logger().warn(
                        '等 %.0f s 没等到 /amcl_pose，直接按真值 %s 播种'
                        % (self.probe_timeout, self.fmt(self.truth())))
                    st = SEED

            elif st == SEED:
                if sent < self.seed_burst and now - last_pub >= self.seed_interval:
                    last_pub = now
                    sent += 1
                    self.publish_pose()
                if sent >= self.seed_burst:
                    t_phase = now
                    asked_nomotion = False
                    st = VERIFY

            elif st == VERIFY:
                asked_nomotion = self.ask_nomotion(asked_nomotion)
                e = self.loc_err()
                if (self.amcl_t > t_phase and e is not None
                        and not self.off_pose(e, self.verify_tol)):
                    self.get_logger().info(
                        '播种生效：/amcl_pose 与真值差 %s，已对齐' % self.fmt_err(e))
                    attempts = 0
                    viol_since, last_watch = None, now
                    st = WATCH
                elif now - t_phase >= self.verify_timeout:
                    if attempts < self.max_attempts:
                        self.get_logger().warn(
                            '播种后 %g s 没等到对齐（/amcl_pose 差 %s），再来一次'
                            % (self.verify_timeout,
                               self.fmt_err(e) if e is not None else '未知'))
                        attempts += 1
                        sent = 0
                        last_pub = -1e9
                        st = SEED
                    else:
                        self.get_logger().warn(
                            '播了 %d 次都没对齐，放弃自动播种 %.0f s；'
                            '请确认 AMCL 是否正常，或用 rviz 的 2D Pose Estimate 给一次'
                            % (attempts, self.watch_backoff))
                        attempts = 0
                        viol_since = None
                        watch_ready = now + self.watch_backoff
                        st = WATCH

            else:  # WATCH：待命 + 常驻盯梢。AMCL 一旦离开激活态（重启/关门），下次激活重走一遍
                if state != State.PRIMARY_STATE_ACTIVE:
                    self.get_logger().info('AMCL 不在激活态了，等它下次激活再播种')
                    attempts = 0
                    viol_since = None
                    st = WAIT
                elif not self.watch_enabled:
                    pass
                elif now - last_watch >= self.watch_interval:
                    last_watch = now
                    if now - self.amcl_t > self.watch_stale:
                        # 停着时 AMCL 不发 /amcl_pose（只在动的时候更新），逼它出一帧
                        self.ask_nomotion(False)
                    e = self.loc_err()
                    if e is not None and self.off_pose(e, self.watch_xy_tol):
                        if viol_since is None:
                            viol_since = now
                        elif (now - viol_since >= self.watch_persist
                                and now >= watch_ready):
                            attempts = 1
                            sent = 0
                            last_pub = -1e9
                            viol_since = None
                            self.get_logger().warn(
                                '看门狗：AMCL 认为 %s，同时刻真值 %s，差 %s，'
                                '重新播种' % (self.fmt(self.belief()),
                                          self.fmt(self.truth()), self.fmt_err(e)))
                            st = SEED
                    else:
                        viol_since = None


def main():
    rclpy.init()
    node = AmclSeedPose()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 走前者；launch 收尾/`timeout` 发的 SIGTERM 会在 spin 里抛后者。
        # 不接住的话，用户 Ctrl-C 一次就在 launch 日志里留一段 traceback。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
