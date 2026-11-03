# -*- coding: utf-8 -*-
"""maze.py —— 迷宫的数据结构、随机生成、BFS 求解与导出（纯 Python，不依赖 ROS）。

坐标约定（整个实验统一，第 2/3 步的导航和 A* 都按这个来）：
    迷宫占世界坐标 [0, cols] × [0, rows] 米，每格 1 米。
    格子 (i, j)：i 沿 x 向右，j 沿 y 向上，i∈[0,cols-1]，j∈[0,rows-1]。
    格子 (i, j) 的中心 = (i + 0.5, j + 0.5)。
    起点格 (0, 0) → 世界坐标 (0.5, 0.5)；终点格 (cols-1, rows-1)。

墙的表示：
    h_walls[j][i]：水平墙，位于 y=j，覆盖 x∈[i,i+1]
                   j∈[0, rows]（rows+1 排），i∈[0, cols-1]
    v_walls[j][i]：垂直墙，位于 x=i，覆盖 y∈[j,j+1]
                   i∈[0, cols]（cols+1 排），j∈[0, rows-1]
    True = 有墙，False = 已凿通。

生成算法：随机 DFS（递归回溯）。它生成的是一棵"生成树"，
也就是完美迷宫——任意两格之间恰好一条路，所以起点到终点必然连通。
同一个 seed 永远得到同一张迷宫。
"""

import heapq
import json
import random
from collections import deque

CELL_SIZE = 1.0
WALL_HEIGHT = 1.0
WALL_THICKNESS = 0.08


class Maze:
    def __init__(self, cols=10, rows=10, seed=0,
                 cell_size=CELL_SIZE, wall_height=WALL_HEIGHT,
                 wall_thickness=WALL_THICKNESS, loops=0):
        self.cols = int(cols)
        self.rows = int(rows)
        self.seed = int(seed)
        self.cell_size = float(cell_size)
        self.wall_height = float(wall_height)
        self.wall_thickness = float(wall_thickness)
        self.loops = int(loops)
        self.loops_removed = 0
        self.rng = random.Random(self.seed)

        self.h_walls = [[True] * self.cols for _ in range(self.rows + 1)]
        self.v_walls = [[True] * (self.cols + 1) for _ in range(self.rows)]

        self._carve()
        if self.loops > 0:
            self._add_loops(self.loops)

    # ------------------------------------------------------------------
    # 生成
    # ------------------------------------------------------------------
    def _carve(self):
        """递归回溯：从 (0,0) 出发，随机走到没访问过的邻居并凿墙。"""
        visited = [[False] * self.cols for _ in range(self.rows)]
        visited[0][0] = True
        stack = [(0, 0)]

        while stack:
            i, j = stack[-1]
            options = []
            if j + 1 < self.rows and not visited[i][j + 1]:
                options.append((i, j + 1, 'N'))
            if j - 1 >= 0 and not visited[i][j - 1]:
                options.append((i, j - 1, 'S'))
            if i + 1 < self.cols and not visited[i + 1][j]:
                options.append((i + 1, j, 'E'))
            if i - 1 >= 0 and not visited[i - 1][j]:
                options.append((i - 1, j, 'W'))

            if not options:            # 死胡同，回溯
                stack.pop()
                continue

            ni, nj, direction = self.rng.choice(options)
            if direction == 'N':
                self.h_walls[j + 1][i] = False
            elif direction == 'S':
                self.h_walls[j][i] = False
            elif direction == 'E':
                self.v_walls[j][i + 1] = False
            else:
                self.v_walls[j][i] = False

            visited[ni][nj] = True
            stack.append((ni, nj))

    def _add_loops(self, n):
        """再随机打通 n 面内墙，制造环路（多条路线）。

        只动内部墙，外围边界永远不拆，所以迷宫不会漏。
        拆墙只会增加连通性，起点到终点仍然可达。
        """
        candidates = []
        for j in range(1, self.rows):
            for i in range(self.cols):
                if self.h_walls[j][i]:
                    candidates.append(('h', j, i))
        for j in range(self.rows):
            for i in range(1, self.cols):
                if self.v_walls[j][i]:
                    candidates.append(('v', j, i))
        self.rng.shuffle(candidates)
        for kind, j, i in candidates[:n]:
            if kind == 'h':
                self.h_walls[j][i] = False
            else:
                self.v_walls[j][i] = False
            self.loops_removed += 1

    # ------------------------------------------------------------------
    # 查询 / 求解
    # ------------------------------------------------------------------
    def neighbors(self, cell):
        """返回与 cell 连通（中间没墙）的相邻格子。"""
        i, j = cell
        out = []
        if j + 1 < self.rows and not self.h_walls[j + 1][i]:
            out.append((i, j + 1))
        if j - 1 >= 0 and not self.h_walls[j][i]:
            out.append((i, j - 1))
        if i + 1 < self.cols and not self.v_walls[j][i + 1]:
            out.append((i + 1, j))
        if i - 1 >= 0 and not self.v_walls[j][i]:
            out.append((i - 1, j))
        return out

    def solve(self, start=(0, 0), goal=None):
        """BFS 求最短路径，返回格子列表（含起点和终点）；不连通返回 None。

        完美迷宫里路径唯一，BFS 求出来的同时也是唯一路。
        """
        if goal is None:
            goal = (self.cols - 1, self.rows - 1)
        start = tuple(start)
        goal = tuple(goal)

        parent = {start: None}
        queue = deque([start])
        while queue:
            cur = queue.popleft()
            if cur == goal:
                break
            for nxt in self.neighbors(cur):
                if nxt not in parent:
                    parent[nxt] = cur
                    queue.append(nxt)

        if goal not in parent:
            return None

        route = []
        cur = goal
        while cur is not None:
            route.append(cur)
            cur = parent[cur]
        return route[::-1]

    def inside(self, cell):
        i, j = cell
        return 0 <= i < self.cols and 0 <= j < self.rows

    def astar(self, start=(0, 0), goal=None):
        """A* 求最短路（4 邻接，每步代价 1，曼哈顿启发式）。

        在单位代价的网格上，曼哈顿距离是可采纳且一致的启发式，
        所以 A* 得到的路径长度一定等于 BFS 的最短路长度。
        返回格子列表；不连通返回 None。
        """
        if goal is None:
            goal = (self.cols - 1, self.rows - 1)
        start, goal = tuple(start), tuple(goal)
        if not self.inside(start) or not self.inside(goal):
            return None
        if start == goal:
            return [start]

        def h(c):
            return abs(c[0] - goal[0]) + abs(c[1] - goal[1])

        g = {start: 0}
        parent = {start: None}
        closed = set()
        counter = 0
        heap = [(h(start), counter, start)]

        while heap:
            _, _, cur = heapq.heappop(heap)
            if cur == goal:
                break
            if cur in closed:
                continue
            closed.add(cur)
            for nxt in self.neighbors(cur):
                ng = g[cur] + 1
                if nxt not in g or ng < g[nxt]:
                    g[nxt] = ng
                    parent[nxt] = cur
                    counter += 1
                    heapq.heappush(heap, (ng + h(nxt), counter, nxt))

        if goal not in parent:
            return None
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = parent[cur]
        return path[::-1]

    def cell_center(self, cell):
        i, j = cell
        return ((i + 0.5) * self.cell_size, (j + 0.5) * self.cell_size)

    def route_points(self, route):
        return [self.cell_center(c) for c in route]

    def wall_count(self):
        h = sum(row.count(True) for row in self.h_walls)
        v = sum(row.count(True) for row in self.v_walls)
        return h + v

    # ------------------------------------------------------------------
    # 导出：SDF world
    # ------------------------------------------------------------------
    def _box_link(self, name, x, y, z, sx, sy, sz):
        return (
            '      <link name="%s">\n'
            '        <pose>%.4f %.4f %.4f 0 0 0</pose>\n'
            '        <collision name="col">\n'
            '          <geometry><box><size>%.4f %.4f %.4f</size></box></geometry>\n'
            '        </collision>\n'
            '        <visual name="vis">\n'
            '          <geometry><box><size>%.4f %.4f %.4f</size></box></geometry>\n'
            '          <material>\n'
            '            <ambient>0.35 0.35 0.40 1</ambient>\n'
            '            <diffuse>0.45 0.45 0.50 1</diffuse>\n'
            '            <specular>0.1 0.1 0.1 1</specular>\n'
            '          </material>\n'
            '        </visual>\n'
            '      </link>\n'
        ) % (name, x, y, z, sx, sy, sz, sx, sy, sz)

    def to_sdf(self, world_name='default'):
        """导出成自包含的 Gazebo world（地面和太阳都内联，不引用 model://）。"""
        t = self.wall_thickness
        h = self.wall_height
        links = []

        # 水平墙：y=j 处，横跨 [i, i+1]；长度加 t，让转角处互相搭上
        for j in range(self.rows + 1):
            for i in range(self.cols):
                if self.h_walls[j][i]:
                    links.append(self._box_link(
                        'h_%d_%d' % (j, i), (i + 0.5) * self.cell_size,
                        j * self.cell_size, h / 2.0,
                        self.cell_size + t, t, h))

        # 垂直墙：x=i 处，跨 [j, j+1]
        for j in range(self.rows):
            for i in range(self.cols + 1):
                if self.v_walls[j][i]:
                    links.append(self._box_link(
                        'v_%d_%d' % (j, i), i * self.cell_size,
                        (j + 0.5) * self.cell_size, h / 2.0,
                        t, self.cell_size + t, h))

        head = """<?xml version="1.0" ?>
<!-- 由 maze_nav 自动生成：cols=%d rows=%d seed=%d -->
<sdf version="1.6">
  <world name="%s">

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 地面：内联，不引用 model://ground_plane，避免模型库路径问题 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 提供 /gazebo/set_entity_state（一键回起点）和 /gazebo/model_states（真值位姿） -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>30.0</update_rate>
    </plugin>

    <model name="maze">
      <static>true</static>
""" % (self.cols, self.rows, self.seed, world_name)

        return head + ''.join(links) + """    </model>

  </world>
</sdf>
"""

    # ------------------------------------------------------------------
    # 导出：JSON / PNG
    # ------------------------------------------------------------------
    def to_dict(self, start=(0, 0), goal=None):
        if goal is None:
            goal = (self.cols - 1, self.rows - 1)
        route = self.solve(start, goal)
        return {
            'cols': self.cols,
            'rows': self.rows,
            'seed': self.seed,
            'cell_size': self.cell_size,
            'wall_height': self.wall_height,
            'wall_thickness': self.wall_thickness,
            'loops': self.loops,
            'loops_removed': self.loops_removed,
            'start': list(start),
            'goal': list(goal),
            'h_walls': self.h_walls,
            'v_walls': self.v_walls,
            'route': [list(c) for c in route] if route else None,
            'route_points': [list(p) for p in self.route_points(route)] if route else None,
        }

    def render_png(self, path, route=None, px=56, margin=28):
        """把迷宫画成 PNG（俯视图），用来在终端外快速核对。"""
        from PIL import Image, ImageDraw

        width = margin * 2 + int(self.cols * self.cell_size * px)
        height = margin * 2 + int(self.rows * self.cell_size * px)
        img = Image.new('RGB', (width, height), (250, 250, 250))
        draw = ImageDraw.Draw(img)

        def X(x):
            return margin + x * px

        def Y(y):
            return margin + (self.rows - y) * px

        t = max(3, int(self.wall_thickness * px))

        # 先画路径，再画墙，墙会盖住路径两端
        if route:
            pts = [self.cell_center(c) for c in route]
            draw.line([(X(x), Y(y)) for x, y in pts], fill=(30, 120, 220), width=5)

        for j in range(self.rows + 1):
            for i in range(self.cols):
                if self.h_walls[j][i]:
                    y = Y(j)
                    draw.rectangle([X(i) - t // 2, y - t // 2,
                                    X(i + 1) + t // 2, y + t // 2], fill=(40, 40, 45))
        for j in range(self.rows):
            for i in range(self.cols + 1):
                if self.v_walls[j][i]:
                    x = X(i)
                    draw.rectangle([x - t // 2, Y(j), x + t // 2, Y(j + 1)], fill=(40, 40, 45))

        sx, sy = self.cell_center((0, 0))
        gx, gy = self.cell_center((self.cols - 1, self.rows - 1))
        r = max(6, px // 6)
        draw.ellipse([X(sx) - r, Y(sy) - r, X(sx) + r, Y(sy) + r], fill=(40, 170, 90))
        draw.ellipse([X(gx) - r, Y(gy) - r, X(gx) + r, Y(gy) + r], fill=(220, 70, 70))

        img.save(path)
        return path
