#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""qt_goal_gui.py —— Qt 控制台：点格子设目标，看 A* 路径和车的位置。

界面：
    · 左：迷宫/网格、A* 路径（橙色）、小车、轨迹、目标
    · 右：位置/朝向/目标/距离/状态，停止、回起点重跑、清除轨迹

话题：
    订阅 /gazebo/model_states   Gazebo 真值位姿（有迷宫时有）
    订阅 /odom                 空场地时的位姿回退
    订阅 /plan                  A* 路径
    订阅 /nav_status
    发布 /goal_pose, /nav_cancel, /reset

用法：
    ros2 run maze_nav qt_goal_gui --seed 42
    ros2 run maze_nav qt_goal_gui            # 无迷宫，空场地
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from PyQt5.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtCore import QRectF
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (QApplication, QCheckBox, QHBoxLayout, QLabel,
                             QMainWindow, QPushButton, QVBoxLayout, QWidget)
from rclpy.node import Node
from std_msgs.msg import Empty, String

from maze_nav.maze import Maze


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapCanvas(QWidget):
    goalClicked = pyqtSignal(float, float)

    def __init__(self, xmin, xmax, ymin, ymax, parent=None):
        super().__init__(parent)
        self.xmin, self.xmax = xmin, xmax
        self.ymin, self.ymax = ymin, ymax
        self.margin = 26
        self.pose = None
        self.goal = None
        self.plan = []
        self.trail = []
        self.maze = None
        self.map_image = None
        self.map_bounds = None
        self.show_trail = True
        self.setMinimumSize(560, 560)

    # -- 坐标变换 --
    def _scale(self):
        sw = max(1, self.width() - 2 * self.margin)
        sh = max(1, self.height() - 2 * self.margin)
        return min(sw / (self.xmax - self.xmin), sh / (self.ymax - self.ymin))

    def _origin(self):
        s = self._scale()
        sw = self.width() - 2 * self.margin
        sh = self.height() - 2 * self.margin
        return (self.margin + (sw - (self.xmax - self.xmin) * s) / 2,
                self.margin + (sh - (self.ymax - self.ymin) * s) / 2)

    def w2s(self, x, y):
        s = self._scale()
        ox, oy = self._origin()
        return (ox + (x - self.xmin) * s,
                self.height() - oy - (y - self.ymin) * s)

    def s2w(self, sx, sy):
        s = self._scale()
        ox, oy = self._origin()
        return (self.xmin + (sx - ox) / s,
                self.ymin + (self.height() - oy - sy) / s)

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self.goalClicked.emit(*self.s2w(ev.x(), ev.y()))

    # -- 绘制 --
    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(28, 30, 36))
        s = self._scale()

        # 网格
        p.setPen(QPen(QColor(50, 54, 64), 1))
        x = math.ceil(self.xmin)
        while x <= self.xmax:
            p.drawLine(QPointF(*self.w2s(x, self.ymin)), QPointF(*self.w2s(x, self.ymax)))
            x += 1
        y = math.ceil(self.ymin)
        while y <= self.ymax:
            p.drawLine(QPointF(*self.w2s(self.xmin, y)), QPointF(*self.w2s(self.xmax, y)))
            y += 1

        # 探索得到的占据栅格
        if self.map_image is not None and self.map_bounds is not None:
            x0, y0, res, w, h = self.map_bounds
            p.drawImage(QRectF(QPointF(*self.w2s(x0, y0 + h * res)),
                               QPointF(*self.w2s(x0 + w * res, y0))),
                        self.map_image)

        # 迷墙
        if self.maze is not None:
            p.setPen(QPen(QColor(150, 156, 172), max(3, int(0.07 * s))))
            for j in range(self.maze.rows + 1):
                for i in range(self.maze.cols):
                    if self.maze.h_walls[j][i]:
                        p.drawLine(QPointF(*self.w2s(i, j)),
                                   QPointF(*self.w2s(i + 1, j)))
            for j in range(self.maze.rows):
                for i in range(self.maze.cols + 1):
                    if self.maze.v_walls[j][i]:
                        p.drawLine(QPointF(*self.w2s(i, j)),
                                   QPointF(*self.w2s(i, j + 1)))

        # 坐标轴
        p.setPen(QPen(QColor(105, 110, 125), 1))
        if self.ymin <= 0 <= self.ymax:
            p.drawLine(QPointF(*self.w2s(self.xmin, 0)), QPointF(*self.w2s(self.xmax, 0)))
        if self.xmin <= 0 <= self.xmax:
            p.drawLine(QPointF(*self.w2s(0, self.ymin)), QPointF(*self.w2s(0, self.ymax)))

        p.setPen(QColor(150, 155, 170))
        p.setFont(QFont('DejaVu Sans', 9))
        p.drawText(8, 18, '1 格 = 1 m   左键点击 = 设目标')

        # A* 路径
        if len(self.plan) > 1:
            p.setPen(QPen(QColor(255, 170, 60), 3))
            for a, b in zip(self.plan[:-1], self.plan[1:]):
                p.drawLine(QPointF(*self.w2s(*a)), QPointF(*self.w2s(*b)))

        # 轨迹
        if self.show_trail and len(self.trail) > 1:
            p.setPen(QPen(QColor(60, 170, 220), 2))
            for a, b in zip(self.trail[:-1], self.trail[1:]):
                p.drawLine(QPointF(*self.w2s(*a)), QPointF(*self.w2s(*b)))

        # 目标
        if self.goal is not None:
            gx, gy = self.w2s(*self.goal)
            p.setPen(QPen(QColor(235, 80, 80), 3))
            r = 8
            p.drawEllipse(QPointF(gx, gy), r, r)
            p.drawLine(QPointF(gx - r - 4, gy), QPointF(gx + r + 4, gy))
            p.drawLine(QPointF(gx, gy - r - 4), QPointF(gx, gy + r + 4))

        # 小车
        if self.pose is not None:
            x, y, yaw = self.pose
            cx, cy = self.w2s(x, y)
            p.save()
            p.translate(cx, cy)
            p.rotate(-math.degrees(yaw))
            size = max(10.0, 0.30 * s)
            tri = QPolygonF([QPointF(size, 0),
                             QPointF(-0.7 * size, 0.55 * size),
                             QPointF(-0.7 * size, -0.55 * size)])
            p.setBrush(QColor(80, 220, 140))
            p.setPen(QPen(QColor(20, 120, 70), 2))
            p.drawPolygon(tri)
            p.restore()


class GuiNode(Node):
    def __init__(self):
        super().__init__('qt_goal_gui')
        self.pose_gz = None
        self.pose_odom = None
        self.status = '待命'
        self.goal = None
        self.plan = []

        self.create_subscription(ModelStates, 'gazebo/model_states', self._on_states, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(String, 'nav_status', self._on_status, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.map = None
        self.create_subscription(OccupancyGrid, 'map', self._on_map, 10)

        self._pub_goal = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self._pub_cancel = self.create_publisher(Empty, 'nav_cancel', 10)
        self._pub_reset = self.create_publisher(Empty, 'reset', 10)

    # -- 回调 --
    def _on_states(self, msg):
        try:
            idx = list(msg.name).index('burger')
        except ValueError:
            return
        p = msg.pose[idx].position
        self.pose_gz = (p.x, p.y, yaw_from_quaternion(msg.pose[idx].orientation))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose_odom = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def _on_status(self, msg):
        self.status = msg.data

    def _on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info('收到目标 (%.2f, %.2f)' % self.goal)

    def _on_plan(self, msg):
        self.plan = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

    def _on_map(self, msg):
        self.map = msg

    @property
    def pose(self):
        return self.pose_gz if self.pose_gz is not None else self.pose_odom

    # -- 命令 --
    def send_goal(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.goal = (float(x), float(y))
        self._pub_goal.publish(msg)

    def cancel(self):
        self._pub_cancel.publish(Empty())

    def reset(self):
        self._pub_reset.publish(Empty())


class MainWindow(QMainWindow):
    def __init__(self, node, args, maze):
        super().__init__()
        self.node = node
        self.maze = maze
        self.setWindowTitle('Burger 迷宫导航 — A* 最短路')

        if args.xmin is None:
            args.xmin = -0.5 if maze else -1.0
        if args.xmax is None:
            args.xmax = (maze.cols + 0.5) if maze else 11.0
        if args.ymin is None:
            args.ymin = -0.5 if maze else -1.0
        if args.ymax is None:
            args.ymax = (maze.rows + 0.5) if maze else 11.0

        self.canvas = MapCanvas(args.xmin, args.xmax, args.ymin, args.ymax)
        self.canvas.maze = maze
        self.explore = getattr(args, 'explore', False)
        self.goal_point = (args.goal_x, args.goal_y)
        if self.explore:
            self.canvas.goal = self.goal_point
        self.canvas.goalClicked.connect(self.on_click)

        panel = QVBoxLayout()
        self.lbl_pose = QLabel('位置: —')
        self.lbl_yaw = QLabel('朝向: —')
        self.lbl_goal = QLabel('目标: —')
        self.lbl_dist = QLabel('距离: —')
        self.lbl_plan = QLabel('路径: —')
        self.lbl_status = QLabel('状态: 待命')
        for w in (self.lbl_pose, self.lbl_yaw, self.lbl_goal,
                  self.lbl_dist, self.lbl_plan, self.lbl_status):
            w.setStyleSheet('color:#d8dbe2; font-size:13px; padding:2px;')
            panel.addWidget(w)
        if self.explore:
            self.lbl_goal.setText('目标: (%.1f, %.1f)' % self.goal_point)

        panel.addSpacing(10)
        self.btn_reset = QPushButton('回到起点并重跑')
        self.btn_stop = QPushButton('停止')
        self.btn_clear = QPushButton('清除轨迹')
        self.btn_reset.clicked.connect(self.node.reset)
        self.btn_stop.clicked.connect(self.node.cancel)
        self.btn_clear.clicked.connect(self.on_clear)
        self.btn_reset.setVisible(not getattr(args, 'explore', False))
        panel.addWidget(self.btn_reset)
        panel.addWidget(self.btn_stop)
        panel.addWidget(self.btn_clear)

        self.chk_trail = QCheckBox('显示轨迹')
        self.chk_trail.setChecked(True)
        self.chk_trail.stateChanged.connect(
            lambda: setattr(self.canvas, 'show_trail', self.chk_trail.isChecked()))
        panel.addWidget(self.chk_trail)

        panel.addStretch(1)
        hint = QLabel('左键点地图 = 设目标\n橙色线 = A* 最短路')
        hint.setStyleSheet('color:#8b91a1; font-size:12px;')
        panel.addWidget(hint)

        right = QWidget()
        right.setLayout(panel)
        right.setFixedWidth(230)
        right.setStyleSheet('background:#22242c;')

        central = QWidget()
        lay = QHBoxLayout(central)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.canvas, 1)
        lay.addWidget(right, 0)
        self.setCentralWidget(central)
        self.resize(940, 660)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)

    def _update_map_image(self):
        msg = self.node.map
        w, h = msg.info.width, msg.info.height
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if getattr(self, '_map_stamp', None) == stamp:
            return
        self._map_stamp = stamp
        img = QImage(w, h, QImage.Format_RGB32)
        data = msg.data
        for j in range(h):
            row = j * w
            y = h - 1 - j
            for i in range(w):
                v = data[row + i]
                if v == 100:
                    img.setPixel(i, y, 0xFF23262E)
                elif v == 0:
                    img.setPixel(i, y, 0xFFC8C8CE)
                else:
                    img.setPixel(i, y, 0xFF4A4E5A)
        self.canvas.map_image = img
        self.canvas.map_bounds = (msg.info.origin.position.x,
                                  msg.info.origin.position.y,
                                  msg.info.resolution, w, h)

    def on_click(self, x, y):
        if getattr(self, 'explore', False):
            return
        if self.maze is not None:
            if not (0 <= x < self.maze.cols and 0 <= y < self.maze.rows):
                return
            x, y = self.maze.cell_center((int(x), int(y)))
        self.canvas.goal = (x, y)
        self.node.send_goal(x, y)

    def on_clear(self):
        self.canvas.trail = []
        self.canvas.update()

    def refresh(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

        pose = self.node.pose
        if pose is not None:
            self.canvas.pose = pose
            self.lbl_pose.setText('位置: (%.2f, %.2f)' % pose[:2])
            self.lbl_yaw.setText('朝向: %.0f°' % math.degrees(pose[2]))
            if self.canvas.show_trail:
                pt = (pose[0], pose[1])
                if not self.canvas.trail or \
                        math.hypot(pt[0] - self.canvas.trail[-1][0],
                                   pt[1] - self.canvas.trail[-1][1]) > 0.02:
                    self.canvas.trail.append(pt)

        if self.node.goal is not None:
            self.canvas.goal = self.node.goal
            self.lbl_goal.setText('目标: (%.2f, %.2f)' % self.node.goal)

        if self.node.map is not None and getattr(self, 'explore', False):
            self._update_map_image()

        if getattr(self, 'explore', False):
            self.canvas.goal = self.goal_point

        self.canvas.plan = self.node.plan
        if self.node.plan:
            self.lbl_plan.setText('路径: %d 格 / %d 步'
                                  % (len(self.node.plan), len(self.node.plan) - 1))
        else:
            self.lbl_plan.setText('路径: —')

        if pose is not None:
            tx, ty = self.goal_point if getattr(self, 'explore', False) else (
                self.node.plan[-1] if self.node.plan else (pose[0], pose[1]))
            self.lbl_dist.setText('距离: %.2f m' % math.hypot(tx - pose[0], ty - pose[1]))

        self.lbl_status.setText('状态: %s' % self.node.status)
        self.canvas.update()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=-1, help='迷宫种子；-1 表示无迷宫')
    parser.add_argument('--explore', action='store_true', help='未知迷宫探索模式（画 /map）')
    parser.add_argument('--goal-x', type=float, default=9.5)
    parser.add_argument('--goal-y', type=float, default=9.5)
    parser.add_argument('--cols', type=int, default=10)
    parser.add_argument('--rows', type=int, default=10)
    parser.add_argument('--xmin', type=float, default=None)
    parser.add_argument('--xmax', type=float, default=None)
    parser.add_argument('--ymin', type=float, default=None)
    parser.add_argument('--ymax', type=float, default=None)
    parser.add_argument('--click', nargs=2, type=float, metavar=('X', 'Y'),
                        help='（自测用）启动后模拟一次鼠标点击')
    parser.add_argument('--click-delay', type=float, default=2.0)
    parser.add_argument('--screenshot', default='', help='（自测用）截图后退出')
    parser.add_argument('--screenshot-delay', type=float, default=3.0)
    args, ros_args = parser.parse_known_args(argv)

    maze = Maze(args.cols, args.rows, args.seed) if args.seed >= 0 else None

    rclpy.init(args=ros_args)
    node = GuiNode()

    app = QApplication(sys.argv[:1])
    win = MainWindow(node, args, maze)
    win.show()

    if args.click:
        QTimer.singleShot(int(args.click_delay * 1000),
                          lambda: win.on_click(args.click[0], args.click[1]))

    if args.screenshot:
        def shot():
            win.grab().save(args.screenshot)
            app.quit()
        QTimer.singleShot(int(args.screenshot_delay * 1000), shot)

    app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
