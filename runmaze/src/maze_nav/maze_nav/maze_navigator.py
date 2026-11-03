# -*- coding: utf-8 -*-
"""maze_navigator.py —— 迷宫里的 A* 规划 + 路径跟随 + 一键回起点。

流程：
    /goal_pose（世界坐标）→ 换算成格子 → A* 求最短路 →
    发布 /plan（nav_msgs/Path）→ 逐格心跟随 → 到终点停车。

位姿来源是 Gazebo 真值 /gazebo/model_states（不是 /odom），
因为"一键回起点"是把模型瞬移回去，里程计不会跟着清零。

话题：
    订阅 /gazebo/model_states   真值位姿
    订阅 /goal_pose             目标（maze 世界坐标，米）
    订阅 /nav_cancel            停止
    订阅 /reset                 瞬移回起点并重跑当前目标
    发布 /plan                  A* 路径（每个格心一个点）
    发布 /cmd_vel               速度
    发布 /nav_status            状态文本
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Empty, String

from maze_nav.maze import Maze


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MazeNavigator(Node):
    def __init__(self):
        super().__init__('maze_navigator')

        self.declare_parameter('seed', 0)
        self.declare_parameter('cols', 10)
        self.declare_parameter('rows', 10)
        self.declare_parameter('entity', 'burger')
        self.declare_parameter('max_linear', 0.28)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('wp_tolerance', 0.15)

        cols = self.get_parameter('cols').value
        rows = self.get_parameter('rows').value
        self.maze = Maze(cols, rows, self.get_parameter('seed').value)
        self.entity = self.get_parameter('entity').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.wp_tol = self.get_parameter('wp_tolerance').value

        self.pos = None            # (x, y, yaw) 世界坐标
        self.goal_cell = None
        self.path = []
        self.wp = 0
        self.state = '待命'

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_plan = self.create_publisher(Path, 'plan', 10)
        self.pub_status = self.create_publisher(String, 'nav_status', 10)

        self.create_subscription(ModelStates, 'gazebo/model_states', self.on_states, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)
        self.create_subscription(Empty, 'nav_cancel', self.on_cancel, 10)
        self.create_subscription(Empty, 'reset', self.on_reset, 10)

        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.create_timer(0.05, self.control)

        self.get_logger().info(
            '迷宫 seed=%d %dx%d，等 /goal_pose' % (self.maze.seed, cols, rows))

    # ------------------------------------------------------------------
    # 位姿
    # ------------------------------------------------------------------
    def on_states(self, msg):
        try:
            idx = list(msg.name).index(self.entity)
        except ValueError:
            return
        p = msg.pose[idx].position
        self.pos = (p.x, p.y, yaw_from_quaternion(msg.pose[idx].orientation))

    def cell_of(self, x, y):
        i = int(math.floor(x))
        j = int(math.floor(y))
        return (min(max(i, 0), self.maze.cols - 1),
                min(max(j, 0), self.maze.rows - 1))

    def center(self, cell):
        return self.maze.cell_center(cell)

    # ------------------------------------------------------------------
    # 目标与规划
    # ------------------------------------------------------------------
    def on_goal(self, msg):
        cell = self.cell_of(msg.pose.position.x, msg.pose.position.y)
        self.goal_cell = cell
        self.get_logger().info('目标格 %s（世界坐标 %.2f, %.2f）'
                               % (cell, *self.center(cell)))
        self.plan_and_start()

    def plan_and_start(self):
        if self.goal_cell is None:
            return
        if self.pos is None:
            self.set_state('等待定位')
            return

        start = self.cell_of(self.pos[0], self.pos[1])
        path = self.maze.astar(start, self.goal_cell)
        if not path:
            self.path = []
            self.set_state('无路可走')
            return

        self.path = path
        self.wp = 0
        self.publish_plan(path)
        self.set_state('导航中')
        self.get_logger().info(
            'A*: %s -> %s，%d 格 / %d 步'
            % (start, self.goal_cell, len(path), len(path) - 1))

    def publish_plan(self, path):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        for cell in path:
            ps = PoseStamped()
            ps.header = msg.header
            x, y = self.center(cell)
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_plan.publish(msg)

    # ------------------------------------------------------------------
    # 控制
    # ------------------------------------------------------------------
    def control(self):
        twist = Twist()
        if self.state != '导航中' or not self.path or self.pos is None:
            self.pub_cmd.publish(twist)
            return

        target = self.center(self.path[self.wp])
        dx = target[0] - self.pos[0]
        dy = target[1] - self.pos[1]
        dist = math.hypot(dx, dy)

        tol = self.goal_tol if self.wp >= len(self.path) - 1 else self.wp_tol
        if dist < tol:
            if self.wp >= len(self.path) - 1:
                self.path = []
                self.pub_cmd.publish(Twist())
                self.set_state('已到达')
                self.publish_status('已到达')
                return
            self.wp += 1
            target = self.center(self.path[self.wp])
            dx = target[0] - self.pos[0]
            dy = target[1] - self.pos[1]
            dist = math.hypot(dx, dy)

        err = normalize_angle(math.atan2(dy, dx) - self.pos[2])

        if abs(err) > 0.6:
            twist.angular.z = max(-self.max_angular,
                                  min(self.max_angular, 1.8 * err))
        else:
            twist.linear.x = min(self.max_linear, 0.9 * dist) * max(0.3, math.cos(err))
            twist.angular.z = max(-self.max_angular,
                                  min(self.max_angular, 1.8 * err))

        self.pub_cmd.publish(twist)
        self.publish_status('导航中 %d/%d' % (self.wp + 1, len(self.path)))

    # ------------------------------------------------------------------
    # 停止 / 回起点
    # ------------------------------------------------------------------
    def on_cancel(self, msg):
        self.path = []
        self.pub_cmd.publish(Twist())
        self.set_state('已停止')

    def on_reset(self, msg):
        self.pub_cmd.publish(Twist())
        self.path = []

        sx, sy = self.center((0, 0))
        if self.teleport(sx, sy):
            self.pos = (sx, sy, 0.0)
            self.get_logger().info('已把 %s 瞬移回起点 (%.2f, %.2f)'
                                   % (self.entity, sx, sy))
        if self.goal_cell is not None:
            self.plan_and_start()
        else:
            self.set_state('待命')

    def teleport(self, x, y, yaw=0.0):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/gazebo/set_entity_state 不可用，无法瞬移')
            return False
        req = SetEntityState.Request()
        req.state.name = self.entity
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.02
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = 'world'
        self.cli.call_async(req)
        return True

    # ------------------------------------------------------------------
    def set_state(self, text):
        self.state = text
        self.publish_status(text)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = MazeNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
