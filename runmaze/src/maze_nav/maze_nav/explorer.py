# -*- coding: utf-8 -*-
"""explorer.py —— 未知迷宫自主探索 + A* 导航（按 1 米格子建图）。

车一开始不知道墙在哪：
    · 每到一个格子中心，用 /scan 看东/北/西/南四个方向 0.8 m 内有没有墙，
      把"这一格和邻居通不通"记进一张图；
    · 贪心地挑"离终点最近、还没去过"的相邻格走；走不通就沿已知路退回
      最近的有未探索邻居的格子（DFS 回溯）；
    · 一旦终点在这张已知图里可达，立刻跑 A* 求最短路并开过去。

为什么不用像素级前沿探索：1 m 宽的走廊里，3 m 雷达会把视线内的空间看穿，
未知区域永远隔着墙、不与空闲区相邻，前沿集合恒为空，车会原地不动。

位姿用 Gazebo 真值；"未知"的只是墙。
"""

import heapq
import math
import random
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

UNKNOWN, FREE, OCC = -1, 0, 1
DIRS = {'E': (1, 0), 'N': (0, 1), 'W': (-1, 0), 'S': (0, -1)}
ANGLES = {'E': 0.0, 'N': math.pi / 2, 'W': math.pi, 'S': -math.pi / 2}


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Explorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')

        self.declare_parameter('cell_size', 1.0)
        self.declare_parameter('cols', 10)
        self.declare_parameter('rows', 10)
        self.declare_parameter('goal_x', 9.5)
        self.declare_parameter('goal_y', 9.5)
        self.declare_parameter('wall_threshold', 0.8)
        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('entity', 'burger')
        self.declare_parameter('map_resolution', 0.10)
        self.declare_parameter('strategy', 'random')   # random=DFS 乱逛；greedy=朝终点
        self.declare_parameter('explore_all', False)   # true=逛完整座迷宫再去终点

        self.cell = self.get_parameter('cell_size').value
        self.cols = self.get_parameter('cols').value
        self.rows = self.get_parameter('rows').value
        self.goal = (self.get_parameter('goal_x').value,
                     self.get_parameter('goal_y').value)
        self.wall_th = self.get_parameter('wall_threshold').value
        self.max_v = self.get_parameter('max_linear').value
        self.max_w = self.get_parameter('max_angular').value
        self.entity = self.get_parameter('entity').value
        self.res = self.get_parameter('map_resolution').value
        self.strategy = self.get_parameter('strategy').value
        self.explore_all = self.get_parameter('explore_all').value
        self.rng = random.Random(0)

        # 已知图：walls[(i,j)] = {'E': True/False, ...}
        self.walls = {}
        self.visited = set()
        self.graph = {}                 # (i,j) -> set(邻居)

        # 栅格图（只用于 Qt 显示）
        self.x0, self.y0 = -1.0, -1.0
        self.nx = int((self.cols + 2) / self.res)
        self.ny = int((self.rows + 2) / self.res)
        self.grid = [[UNKNOWN] * self.nx for _ in range(self.ny)]

        self.pos = None
        self.scan = None
        self.route = []                 # 世界坐标航点
        self.wp = 0
        self.state = '等待传感器'
        self.done = False
        self.last_scan_cell = None
        self._last_pos = None
        self._last_move_t = None
        self.recover_until = 0.0

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 30)
        self.pub_map = self.create_publisher(OccupancyGrid, 'map', 10)
        self.pub_plan = self.create_publisher(Path, 'plan', 10)
        self.pub_status = self.create_publisher(String, 'nav_status', 10)

        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(ModelStates, 'gazebo/model_states', self.on_states, 10)
        self.create_subscription(Empty, 'nav_cancel', self.on_cancel, 10)

        self.create_timer(0.20, self.update_map)
        self.create_timer(0.10, self.control)
        self.create_timer(0.50, self.think)
        self.create_timer(0.50, self.publish_map)

        self.get_logger().info(
            '未知迷宫探索 %dx%d，终点 (%.1f, %.1f)'
            % (self.cols, self.rows, self.goal[0], self.goal[1]))

    # ------------------------------------------------------------------
    def on_scan(self, msg):
        self.scan = msg

    def on_states(self, msg):
        try:
            idx = list(msg.name).index(self.entity)
        except ValueError:
            return
        p = msg.pose[idx].position
        self.pos = (p.x, p.y, yaw_from_quaternion(msg.pose[idx].orientation))

    def on_cancel(self, msg):
        self.route = []
        self.done = True
        self.pub_cmd.publish(Twist())
        self.set_state('已停止')

    # ------------------------------------------------------------------
    # 格子工具
    # ------------------------------------------------------------------
    def cell_of(self, x, y):
        return (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))

    def center(self, c):
        return ((c[0] + 0.5) * self.cell, (c[1] + 0.5) * self.cell)

    def inside(self, c):
        return 0 <= c[0] < self.cols and 0 <= c[1] < self.rows

    def neighbors(self, c):
        return [(c[0] + d[0], c[1] + d[1]) for d in DIRS.values()
                if self.inside((c[0] + d[0], c[1] + d[1]))]

    def is_open(self, c, n):
        """已知通/不通；未知返回 None。"""
        w = self.walls.get(c)
        if w is None:
            return None
        for name, d in DIRS.items():
            if (c[0] + d[0], c[1] + d[1]) == n:
                return not w.get(name, True)
        return None

    # ------------------------------------------------------------------
    # 到了一个新格子：扫四个方向，写入已知图
    # ------------------------------------------------------------------
    def scan_cell(self, c):
        if self.scan is None or self.pos is None:
            return False
        walls = {}
        for name, a0 in ANGLES.items():
            vals = []
            for k, r in enumerate(self.scan.ranges):
                if not math.isfinite(r) or r < self.scan.range_min:
                    continue
                a = self.pos[2] + self.scan.angle_min + k * self.scan.angle_increment
                if abs(normalize_angle(a - a0)) < 0.20 and r < 3.5:
                    vals.append(r)
            # 这一格中心到墙约 0.46 m；开口方向至少 1.46 m
            walls[name] = (min(vals) < self.wall_th) if vals else True
        self.walls[c] = walls
        self.visited.add(c)
        self.graph[c] = {n for n in self.neighbors(c) if not walls[self._dir(c, n)]}
        for n in self.neighbors(c):
            if not walls[self._dir(c, n)]:
                self.graph.setdefault(n, set()).add(c)
        self.last_scan_cell = c
        return True

    @staticmethod
    def _dir(c, n):
        for name, d in DIRS.items():
            if (c[0] + d[0], c[1] + d[1]) == n:
                return name
        return 'E'

    # ------------------------------------------------------------------
    # 已知图上的最短路（A* / BFS 都行，这里统一用 A*）
    # ------------------------------------------------------------------
    def astar(self, start, goal):
        if start not in self.graph or goal not in self.graph:
            return None
        h = lambda c: abs(c[0] - goal[0]) + abs(c[1] - goal[1])
        g = {start: 0}
        parent = {start: None}
        heap = [(h(start), 0, start)]
        closed = set()
        while heap:
            _, _, cur = heapq.heappop(heap)
            if cur == goal:
                break
            if cur in closed:
                continue
            closed.add(cur)
            for n in self.graph.get(cur, ()):
                ng = g[cur] + 1
                if n not in g or ng < g[n]:
                    g[n] = ng
                    parent[n] = cur
                    heapq.heappush(heap, (ng + h(n), 0, n))
        if goal not in parent:
            return None
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = parent[cur]
        return path[::-1]

    def bfs_from(self, start):
        dist = {start: 0}
        queue = [start]
        head = 0
        while head < len(queue):
            cur = queue[head]
            head += 1
            for n in self.graph.get(cur, ()):
                if n not in dist:
                    dist[n] = dist[cur] + 1
                    queue.append(n)
        return dist

    # ------------------------------------------------------------------
    def think(self):
        if self.done or self.pos is None or self.scan is None:
            return
        cur = self.cell_of(*self.pos[:2])
        if not self.inside(cur):
            return

        gi, gj = self.cell_of(*self.goal)
        # 已经在终点
        if (cur == (gi, gj) and math.hypot(self.pos[0] - self.goal[0],
                                           self.pos[1] - self.goal[1]) < 0.20
                and (not self.explore_all or not self.has_frontier())):
            self.done = True
            self.route = []
            self.pub_cmd.publish(Twist())
            self.set_state('已到达终点（探索 %d 格）' % len(self.visited))
            return

        # 没扫过的格子：必须走到格心附近才扫，避免路过格边时把墙看错
        if cur not in self.visited:
            cx, cy = self.center(cur)
            if math.hypot(self.pos[0] - cx, self.pos[1] - cy) > 0.25:
                return
            self.scan_cell(cur)
            self.get_logger().info('扫描 %s: %s' % (cur, self.walls[cur]))

        # explore_all 模式：先把整座迷宫逛完（会有大量回溯），再去终点
        if self.explore_all and self.has_frontier():
            if not self.route:
                self.explore_step(cur)
            return

        # 终点已在已知图里可达 → A* 过去
        if (gi, gj) in self.graph:
            # 已经在去终点的路上就别重复规划，否则每次都会把路线重置回格心，
            # 车会像被橡皮筋拽住一样在格心附近来回摆
            if self.route and self.route[-1] == self.center((gi, gj)):
                return
            path = self.astar(cur, (gi, gj))
            if path:
                self.drive_cells(path, 'A* 前往终点')
                self.get_logger().info(
                    'A* %s -> (%d,%d)，%d 步' % (cur, gi, gj, len(path) - 1))
                return

        if not self.route:
            self.explore_step(cur)

    def has_frontier(self):
        for c in self.visited:
            if any(n not in self.visited for n in self.graph.get(c, ())):
                return True
        return False

    def explore_step(self, cur):
        # 1) 有没去过的开放邻居，挑离终点最近的
        cand = sorted(n for n in self.graph.get(cur, ()) if n not in self.visited)
        if cand:
            if self.strategy == 'greedy':
                nxt = min(cand, key=lambda n: math.hypot(
                    self.center(n)[0] - self.goal[0],
                    self.center(n)[1] - self.goal[1]))
            else:
                nxt = self.rng.choice(cand)
            self.drive_cells([cur, nxt], '探索 %s' % (nxt,))
            self.get_logger().info('探索 %s -> %s，已访问 %d 格'
                                   % (cur, nxt, len(self.visited)))
            return

        # 2) 否则沿已知图退回最近的"还有未探索邻居"的格子
        dist = self.bfs_from(cur)
        best = None
        best_score = 1e9
        for c in self.visited:
            if c not in dist:
                continue
            if any(n not in self.visited for n in self.graph.get(c, ())):
                gx, gy = self.center(c)
                score = dist[c] + 0.5 * math.hypot(gx - self.goal[0],
                                                   gy - self.goal[1])
                if score < best_score:
                    best_score, best = score, c
        if best is None:
            self.set_state('探索完成但没有通路')
            self.done = True
            return
        path = self.astar(cur, best)
        if not path:
            self.set_state('回溯失败')
            self.done = True
            return
        self.drive_cells(path, '回溯 %s' % (best,))

    def mark_blocked(self, cur, target):
        """把 cur->target 这条边标记成墙，防止反复往同一条死路里撞。"""
        for name, d in DIRS.items():
            if (cur[0] + d[0], cur[1] + d[1]) == target:
                if cur in self.walls:
                    self.walls[cur][name] = True
                self.graph.get(cur, set()).discard(target)
                self.graph.get(target, set()).discard(cur)
                return

    def drive_cells(self, cells, label):
        self.route = [self.center(c) for c in cells]
        # 从 0 开始：先把自己拉回当前格心，再走下一格，避免抄近路蹭墙角
        self.wp = 0
        self._last_pos = None
        self._last_move_t = None
        self.publish_plan()
        self.set_state('%s（已探索 %d 格）' % (label, len(self.visited)))

    # ------------------------------------------------------------------
    def control(self):
        twist = Twist()
        if self.done or not self.route or self.pos is None:
            self.pub_cmd.publish(twist)
            return

        while self.wp < len(self.route) - 1:
            tx, ty = self.route[self.wp]
            if math.hypot(tx - self.pos[0], ty - self.pos[1]) < 0.12:
                self.wp += 1
            else:
                break

        now = time.time()
        if now < self.recover_until:
            twist.linear.x = -0.08
            self.pub_cmd.publish(twist)
            return

        if self._last_pos is None or math.hypot(self.pos[0] - self._last_pos[0],
                                                self.pos[1] - self._last_pos[1]) > 0.05:
            self._last_pos = (self.pos[0], self.pos[1])
            self._last_move_t = now
        elif now - self._last_move_t > 3.0:
            self.mark_blocked(self.cell_of(*self.pos[:2]),
                              self.cell_of(*self.route[-1]))
            self.route = []
            self.recover_until = now + 1.0
            self.pub_cmd.publish(Twist())
            self.set_state('卡住，倒退重规划')
            return

        tx, ty = self.route[self.wp]
        dx, dy = tx - self.pos[0], ty - self.pos[1]
        dist = math.hypot(dx, dy)
        if dist < 0.12 and self.wp >= len(self.route) - 1:
            self.route = []
            return

        err = normalize_angle(math.atan2(dy, dx) - self.pos[2])
        if abs(err) > 0.6:
            twist.angular.z = max(-self.max_w, min(self.max_w, 1.8 * err))
        else:
            twist.linear.x = min(self.max_v, 0.9 * dist) * max(0.3, math.cos(err))
            twist.angular.z = max(-self.max_w, min(self.max_w, 1.8 * err))
        self.pub_cmd.publish(twist)

    # ------------------------------------------------------------------
    def update_map(self):
        """激光点云写进占据栅格，纯粹给 Qt 画图用。"""
        if self.pos is None or self.scan is None:
            return
        x, y, yaw = self.pos
        step = 0.06
        for k in range(0, len(self.scan.ranges), 3):
            r = self.scan.ranges[k]
            if not math.isfinite(r) or r < self.scan.range_min:
                continue
            r = min(r, 3.0)
            a = yaw + self.scan.angle_min + k * self.scan.angle_increment
            ca, sa = math.cos(a), math.sin(a)
            for s in range(1, int(r / step)):
                d = s * step
                i = int((x + ca * d - self.x0) / self.res)
                j = int((y + sa * d - self.y0) / self.res)
                if 0 <= i < self.nx and 0 <= j < self.ny and self.grid[j][i] == UNKNOWN:
                    self.grid[j][i] = FREE
            i = int((x + ca * r - self.x0) / self.res)
            j = int((y + sa * r - self.y0) / self.res)
            if 0 <= i < self.nx and 0 <= j < self.ny and r < self.scan.range_max - 0.05:
                self.grid[j][i] = OCC

    def publish_plan(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        for x, y in self.route:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_plan.publish(msg)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.info.resolution = self.res
        msg.info.width = self.nx
        msg.info.height = self.ny
        msg.info.origin.position.x = self.x0
        msg.info.origin.position.y = self.y0
        msg.info.origin.orientation.w = 1.0
        data = []
        for j in range(self.ny):
            for i in range(self.nx):
                v = self.grid[j][i]
                data.append(100 if v == OCC else (0 if v == FREE else -1))
        msg.data = data
        self.pub_map.publish(msg)

    def set_state(self, text):
        if text != self.state:
            self.get_logger().info(text)
        self.state = text
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = Explorer()
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
