# -*- coding: utf-8 -*-
"""go_to_goal.py —— 让 Burger 开到指定的 (x, y) 坐标。

控制律很简单（先转向、再直行，接近时减速）：
    1) 由 /odom 得到当前位姿 (x, y, yaw)；
    2) 算目标方向与车头朝向的夹角 err；
    3) |err| 大 → 原地转；否则边走边微调，距离越近越慢；
    4) 距离 < tolerance → 停车，打印"到达"。

目标可以两种方式给：
    · 启动参数：ros2 run maze_nav go_to_goal --ros-args -p goal_x:=3.0 -p goal_y:=2.0
    · 运行时话题：ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'odom'}, pose: {position: {x: 3.0, y: 2.0}}}"

注意：这是"直着开过去"的局部控制，不会绕墙。迷宫里的避障/最短路是第 3 步 A* 的事。
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from rclpy.node import Node


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        self.declare_parameter('goal_x', float('nan'))   # NaN = 先待命，等 /goal_pose
        self.declare_parameter('goal_y', float('nan'))
        self.declare_parameter('tolerance', 0.08)        # 到达判定（米）
        self.declare_parameter('max_linear', 0.60)       # 最大线速度 m/s
        self.declare_parameter('max_angular', 1.8)       # 最大角速度 rad/s
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.8)
        self.declare_parameter('rotate_threshold', 0.5)  # 大于此角度先原地转
        self.declare_parameter('rate', 20.0)             # 控制频率 Hz

        self.tolerance = self.get_parameter('tolerance').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.k_linear = self.get_parameter('k_linear').value
        self.k_angular = self.get_parameter('k_angular').value
        self.rotate_threshold = self.get_parameter('rotate_threshold').value

        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        self.goal = None if (math.isnan(gx) or math.isnan(gy)) else (gx, gy)
        self.pose = None            # (x, y, yaw)
        self.arrived = False
        self.last_log = 0.0

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_status = self.create_publisher(String, 'nav_status', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)
        self.create_subscription(Empty, 'nav_cancel', self.on_cancel, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.control)

        if self.goal is None:
            self.get_logger().info('待命中：等待 /goal_pose 给目标（容差 %.2f m）' % self.tolerance)
        else:
            self.get_logger().info(
                '目标 (%.2f, %.2f)，容差 %.2f m'
                % (self.goal[0], self.goal[1], self.tolerance))
        self.publish_status('待命' if self.goal is None else '导航中')

    # ------------------------------------------------------------------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (p.x, p.y, yaw)

    def on_goal(self, msg):
        p = msg.pose.position
        self.goal = (p.x, p.y)
        self.arrived = False
        self.publish_status('导航中')
        self.get_logger().info('新目标 (%.2f, %.2f)' % (p.x, p.y))

    def on_cancel(self, msg):
        self.goal = None
        self.arrived = False
        self.pub.publish(Twist())
        self.publish_status('已停止')
        self.get_logger().info('收到取消，停车')

    # ------------------------------------------------------------------
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    # ------------------------------------------------------------------
    def control(self):
        twist = Twist()
        if self.pose is None or self.goal is None:
            self.pub.publish(twist)
            return

        x, y, yaw = self.pose
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        dist = math.hypot(dx, dy)

        if dist < self.tolerance:
            self.pub.publish(twist)
            if not self.arrived:
                self.arrived = True
                self.publish_status('已到达')
                self.get_logger().info(
                    '到达目标 (%.2f, %.2f)，当前位姿 (%.2f, %.2f)'
                    % (self.goal[0], self.goal[1], x, y))
            return

        self.arrived = False
        err = normalize_angle(math.atan2(dy, dx) - yaw)

        if abs(err) > self.rotate_threshold:
            twist.angular.z = max(-self.max_angular,
                                  min(self.max_angular, self.k_angular * err))
        else:
            # 边走边修正；夹角越大速度越低，避免画大弧
            speed = min(self.max_linear, self.k_linear * dist)
            twist.linear.x = speed * max(0.25, math.cos(err))
            twist.angular.z = max(-self.max_angular,
                                  min(self.max_angular, self.k_angular * err))

        self.pub.publish(twist)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log > 1.0:
            self.last_log = now
            self.get_logger().info(
                '距离 %.2f m，朝向误差 %.0f°，当前 (%.2f, %.2f)'
                % (dist, math.degrees(err), x, y))


def main(argv=None):
    rclpy.init(args=argv)
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())      # 退出前停车
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
