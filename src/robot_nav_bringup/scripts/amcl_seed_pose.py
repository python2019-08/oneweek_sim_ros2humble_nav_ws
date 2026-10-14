#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AMCL 播种位姿：AMCL 每次激活时，用里程计真值替掉写死的 initial_pose。

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
量，没必要拿真值去覆盖）。nav2 重启几次就重新播几次，**不需要改动 nav2 的启动命令**。

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
  skip_tol        已经比这个还准就不播了，默认 0.10 m
  verify_tol      播完确认对齐的容差，默认 0.25 m（激光更新后信念有几厘米抖动）
  verify_timeout  播完等对齐的上限，默认 6.0 s
  probe_timeout   激活后等 /amcl_pose 的上限，默认 5.0 s；等不到就直接播
  max_attempts    一次激活最多播几轮，默认 3
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty

WAIT, PROBE, SEED, VERIFY, OFF = 0, 1, 2, 3, 4

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
        self.verify_tol = self.declare_parameter('verify_tol', 0.25).value
        self.verify_timeout = self.declare_parameter('verify_timeout', 6.0).value
        self.probe_timeout = self.declare_parameter('probe_timeout', 5.0).value
        self.max_attempts = self.declare_parameter('max_attempts', 3).value

        self.odom = None
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

    def on_amcl(self, m):
        self.amcl = m
        self.amcl_t = time.monotonic()

    # ---- 取值 ----------------------------------------------------------
    def truth(self):
        p = self.odom.pose.pose
        return (p.position.x, p.position.y, yaw_of(p.orientation))

    def belief(self):
        if self.amcl is None:
            return None
        p = self.amcl.pose.pose
        return (p.position.x, p.position.y, yaw_of(p.orientation))

    def loc_err(self):
        b = self.belief()
        if b is None:
            return None
        t = self.truth()
        return math.hypot(b[0] - t[0], b[1] - t[1])

    @staticmethod
    def fmt(p):
        return '(%+.3f, %+.3f, %+.1f deg)' % (p[0], p[1], math.degrees(p[2]))

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
                    if e <= self.skip_tol:
                        self.get_logger().info(
                            'AMCL 位姿已经对上了（差 %.3f m），不动它' % e)
                        attempts = 0
                        st = OFF
                    else:
                        attempts += 1
                        sent = 0
                        last_pub = -1e9
                        self.get_logger().info(
                            '[%d/%d] AMCL 认为 %s，与真值 %s 差 %.3f m，播种'
                            % (attempts, self.max_attempts, self.fmt(self.belief()),
                               self.fmt(self.truth()), e))
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
                if self.amcl_t > t_phase and e is not None and e <= self.verify_tol:
                    self.get_logger().info(
                        '播种生效：/amcl_pose 与真值差 %.3f m，已对齐' % e)
                    attempts = 0
                    st = OFF
                elif now - t_phase >= self.verify_timeout:
                    if attempts < self.max_attempts:
                        self.get_logger().warn(
                            '播种后 %g s 没等到对齐（/amcl_pose 差 %s），再来一次'
                            % (self.verify_timeout,
                               '%.3f m' % e if e is not None else '未知'))
                        attempts += 1
                        sent = 0
                        last_pub = -1e9
                        st = SEED
                    else:
                        self.get_logger().warn(
                            '播了 %d 次都没对齐，放弃自动播种；请确认 AMCL 是否正常，'
                            '或用 rviz 的 2D Pose Estimate 手动给一次' % attempts)
                        attempts = 0
                        st = OFF

            else:  # OFF：待命。AMCL 一旦离开激活态（重启/关门），下次激活重新走一遍
                if state != State.PRIMARY_STATE_ACTIVE:
                    self.get_logger().info('AMCL 不在激活态了，等它下次激活再播种')
                    attempts = 0
                    st = WAIT


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
