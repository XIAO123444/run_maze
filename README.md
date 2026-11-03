<div align="center">

# 迷宫自主探索与路径规划

**Autonomous Maze Exploration & Path Planning on ROS 2 + Gazebo**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)](https://classic.gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![License](https://img.shields.io/badge/License-Apache--2.0-green)](LICENSE)

给一台 TurtleBot3 Burger 装上「地老鼠」的大脑：**不给地图**，只凭激光雷达与位姿，
一边探测一边建图，用贪心（或随机 DFS）+ BFS 回溯逛未知迷宫，终点一旦进入已知通路就用 A* 冲刺过去。

</div>

![系统架构](assets/architecture.png)

---

## 目录

- [这是什么](#这是什么)
- [效果速览](#效果速览)
- [特性](#特性)
- [环境依赖](#环境依赖)
- [快速开始](#快速开始)
- [运行示例](#运行示例)
- [参数与话题接口](#参数与话题接口)
- [算法一览](#算法一览)
- [实测结果](#实测结果)
- [目录结构](#目录结构)
- [常见问题](#常见问题)
- [开发路线](#开发路线)
- [与课程提交版本的差异](#与课程提交版本的差异)
- [许可与致谢](#许可与致谢)

## 这是什么

「地老鼠走迷宫」（MicroMouse）是经典的机器人竞赛任务：把机器人放进它**事先完全不知道布局**的迷宫，
只能靠自身传感器找路，最终从入口走到出口。

本项目是这一任务的 ROS 2 仿真实现，刻意**不使用 SLAM、不使用预置地图**：

- 迷宫由随机 DFS 现场生成（`seed` 决定一切，同一 seed 永远同一张迷宫）；
- 小车每到一个格心，用 `/scan` 判断「这一格东/北/西/南四面通不通」，写进一张已知格子图；
- 探索阶段用**贪心选点**（挑离终点最近的未访问邻居）或**随机 DFS**，走不动就**BFS 找最近前沿**回溯；
- 终点格一旦进入已知图，立刻用 **A\***（曼哈顿启发式）求最短路并开过去；
- 全程可在自研 Qt 控制台里看到占据栅格、已走轨迹、A* 路径与状态机文本。

它同时也是一份可运行的算法教材：**随机 DFS / BFS / 贪心 / A\*** 四种策略都能在同一个迷宫里切换、对比、量化统计。

## 效果速览

| 指标 | 结果 | 说明 |
|---|---|---|
| 未知迷宫完成率 | 30 / 30 | 10×10 完美迷宫，seed=0..29 |
| 单次探索（seed=42，10×10，8 条环路） | **129 s，探索 25/100 格，1 次回溯，0 次卡死** | Gazebo 实跑，含 28 s 启动 |
| 探索效率 | 平均 4.0 s / 格 | 0.35 m/s 限速 + 90° 转弯 |
| A* 最优性 | BFS 与 A* 步数 100% 一致 | `verify_paths` 批量校验 |
| 策略对比 | 贪心比随机少走 **34%** 的格数 | 30 张迷宫统计 |

![探索过程](assets/gui_explore.png)

> 🎬 **演示视频**：录好后放到 `assets/demo.mp4`（或用 Git LFS / 贴 B 站链接），把这一行改成下面的写法即可嵌入：
> `<video src="assets/demo.mp4" controls width="80%"></video>`

## 特性

- **一条命令跑全套**：`ros2 launch` 在启动阶段现场生成迷宫 `.world`，再依次拉起 Gazebo、机器人、算法节点与可视化。
- **可复现**：迷宫、最短路、出生点全部由 `seed/cols/rows/loops` 决定；换 seed 就是换一张迷宫，其余完全一致。
- **完备探索 + 最优冲刺**：探索保证有限步内发现出口（连通图），冲刺用可采纳启发式的 A\*，路径最短。
- **自带看门狗**：3 s 内位移 < 0.05 m 判定卡死 → 把这条边标记为墙 → 倒退 1 s 重规划，避免顶墙角顶到天荒地老。
- **可观测**：`/map`（占据栅格）、`/plan`（A\* 航点）、`/nav_status`（状态文本），Qt 端实时渲染。
- **可对照**：同一功能包还提供「已知地图 + A\* 导航」(`maze_navigator`) 与「里程计 P 控制」(`go_to_goal`) 两个对照组，便于分层排查问题。
- **可验证**：`verify_paths` 对任意 seed 校验「A\* 路径长度 == BFS 路径长度」且每一步都跨过合法通路。

## 环境依赖

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04（WSL2 亦可，需 WSLg 才能开 Gazebo/Qt 窗口） |
| ROS 2 | Humble Hawksbill |
| 仿真器 | Gazebo 11（`gazebo_ros`） |
| 机器人 | `turtlebot3_gazebo`（TurtleBot3 Burger，LDS-01 雷达） |
| Python | 3.10，`rclpy`、`PyQt5` 5.15 |

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs \
                    ros-humble-turtlebot3-gazebo \
                    python3-pyqt5
```

## 快速开始

```bash
# 1. 克隆（把 <your-github-id> 换成你的 GitHub 用户名）
git clone https://github.com/<your-github-id>/maze-nav.git
cd maze-nav

# 2. 编译
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. 跑未知迷宫自主探索（本项目主线）
ros2 launch maze_nav maze_explore.launch.py seed:=42
```

启动后你会看到：Gazebo 里出现一座 10×10 迷宫、Burger 停在起点格心 `(0.5, 0.5)`；
Qt 控制台里占据栅格逐步扩展、青色轨迹延伸；日志里周期性打印

```
[INFO] [maze_explorer]: 扫描 (0, 0): {'E': True, 'N': False, 'W': True, 'S': True}
[INFO] [maze_explorer]: 探索 (0, 0) -> (0, 1)，已访问 1 格
...
[INFO] [maze_explorer]: A* (9, 8) -> (9,9)，1 步
[INFO] [maze_explorer]: 已到达终点（探索 25 格）
```

> 无图形界面（服务器 / 纯命令行）时加 `gazebo_gui:=false qt_gui:=false`，只跑物理与算法。

## 运行示例

| 命令 | 启动内容 | 用途 |
|---|---|---|
| `ros2 launch maze_nav burger.launch.py` | 空世界 + Burger | 单独验证模型、`/scan`、`/odom` |
| `ros2 launch maze_nav burger_goal.launch.py goal_x:=3.0 goal_y:=2.0` | 上述 + `go_to_goal` | 里程计 P 控制点到点实验 |
| `ros2 launch maze_nav burger_gui.launch.py` | 上述 + Qt 控制台 | 手动点选目标、看轨迹 |
| `ros2 launch maze_nav maze.launch.py seed:=42` | 随机迷宫 + Gazebo | 只验证世界生成 |
| `ros2 launch maze_nav maze_burger.launch.py seed:=42` | 迷宫 + Burger + `maze_navigator` + Qt | **已知地图**下的 A* 导航（对照组） |
| `ros2 launch maze_nav maze_explore.launch.py seed:=42` | 迷宫 + Burger + `maze_explorer` + Qt | **未知迷宫**自主探索（主线） |

其它实用命令：

```bash
# 只用随机种子生成迷宫：world + json + 预览图
ros2 run maze_nav generate_maze --seed 42 --png
ros2 run maze_nav generate_maze --seed 7 --cols 12 --rows 8 --loops 12 --png

# 批量校验 A* 的最短性（默认 100 个种子）
ros2 run maze_nav verify_paths --seeds 200

# 探索策略对比：strategy:=random（默认，演示乱逛+回溯）或 greedy（朝终点，步数更少）
ros2 launch maze_nav maze_explore.launch.py seed:=42 strategy:=greedy
```

## 参数与话题接口

### `maze_explore.launch.py` 启动参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `seed` | `0` | 迷宫随机种子；同一 seed 永远同一张迷宫 |
| `cols` / `rows` | `10` / `10` | 格子数；终点自动取 `(cols-0.5, rows-0.5)` |
| `loops` | `8` | 额外打通的内墙数，>0 产生环路（多解），更容易体现 A* 优势 |
| `entity` | `burger` | Gazebo 模型名，探索节点按它查真值位姿 |
| `gazebo_gui` | `true` | `false` 只启动 `gzserver` |
| `qt_gui` | `true` | `false` 不启动 Qt 控制台 |
| `strategy` | `random` | `random` = 随机选未访问邻居；`greedy` = 选离终点最近的 |
| `explore_all` | `false` | `true` = 先逛完整座迷宫再冲刺（回溯多，适合演示） |

### 核心话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 订阅 | 360 点极坐标测距（QoS: sensor_data） |
| `/odom` | `nav_msgs/Odometry` | 订阅 | 里程计（`go_to_goal` 用） |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | 订阅 | 真值位姿（探索节点用） |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 速度指令 |
| `/map` | `nav_msgs/OccupancyGrid` | 发布 | 0.1 m 占据栅格，仅用于显示 |
| `/plan` | `nav_msgs/Path` | 发布 | A* 航点序列 |
| `/nav_status` | `std_msgs/String` | 发布 | 状态机文本 |
| `/nav_cancel`、`/reset` | `std_msgs/Empty` | 订阅 | 急停 / 回起点重跑 |

### 节点与可执行文件

```bash
ros2 pkg executables maze_nav
# generate_maze  go_to_goal  maze_explorer  maze_navigator  qt_goal_gui  verify_paths
```

## 算法一览

| 算法 | 用在哪 | 一句话原理 | 详情 |
|---|---|---|---|
| 随机 DFS（递归回溯） | 生成迷宫 | 从起点出发随机走向未访问邻居并凿墙，走不动就退栈；结果是**生成树**，任意两格唯一通路 | [docs/algorithm.md](docs/algorithm.md#1-迷宫生成随机-dfs) |
| BFS | 最短路校验、回溯选点 | FIFO 按层扩展，无权图上首次到达即最短；`parent` 回溯出路径 | [docs/algorithm.md](docs/algorithm.md#2-bfs-最短路) |
| 贪心 | 探索选点 | 每步挑离终点最近的未访问邻居（欧氏距离），只看一步 | [docs/algorithm.md](docs/algorithm.md#3-贪心探索) |
| A\* | 冲刺终点 | `f = g + h`，`h` 取曼哈顿距离（可采纳且一致），堆 + `closed` 去重 | [docs/algorithm.md](docs/algorithm.md#4-a-寻路) |
| P 控制 | 航点跟踪 | 分段比例控制：航向误差大时原地转，小时边走边修，接近时降速 | [docs/algorithm.md](docs/algorithm.md#5-p-控制与航点跟踪) |
| 四方向分区 | 判墙 | 每个方向取 ±0.2 rad 扇区内测距最小值，< 0.8 m 判为墙 | [docs/algorithm.md](docs/algorithm.md#6-激光雷达分区) |

![探索状态机](assets/state_machine.png)

## 实测结果

### 1. 迷宫与最短路

| 种子 | 尺寸 | loops | 墙段数 | BFS 步数 | A* 步数 |
|---|---|---|---|---|---|
| 42 | 10×10 | 0 | 121 | 38 | 38 |
| 42 | 10×10 | 8 | 113 | 20 | 20 |
| 3 | 10×10 | 8 | 113 | 34 | 34 |

<p align="center">
  <img src="assets/maze_seed42.png" width="45%" alt="完美迷宫与最短路">
  <img src="assets/maze_loops.png" width="45%" alt="加环路后的迷宫与最短路">
</p>

### 2. BFS 与 A\* 的扩展数对比（同一张 8 环路迷宫）

![BFS vs A*](assets/bfs_vs_astar.png)

BFS 扩展 81 格、A\* 只扩展 40 格，路径长度都是 20 步；在**完美迷宫**（`loops=0`）上两者扩展数是 45 vs 44——
唯一通路把搜索「逼」成一条线，启发式无处发力。这说明启发式的收益取决于图的分支度，而不是算法是否高级。

### 3. 探索策略对比（30 张 10×10 完美迷宫）

| 指标 | 随机 DFS | 贪心（朝终点） | 变化 |
|---|---|---|---|
| 平均移动格数 | 73.5 | **48.4** | −34% |
| 平均探索格数 | 59.7 | **47.1** | −21% |
| 平均回溯次数 | 3.2 | **0.7** | −78% |
| 最好 / 最坏移动格数 | 20 / 160 | 18 / 90 | 方差显著变小 |

![策略对比](assets/strategy_compare.png)

## 目录结构

```
maze-nav/
├── README.md
├── LICENSE
├── .gitignore
├── src/
│   └── maze_nav/
│       ├── package.xml
│       ├── setup.py                  # 6 个 console_scripts 入口
│       ├── setup.cfg
│       ├── resource/maze_nav
│       ├── launch/                   # 6 个 Python launch（见上表）
│       │   ├── burger.launch.py
│       │   ├── burger_goal.launch.py
│       │   ├── burger_gui.launch.py
│       │   ├── maze.launch.py
│       │   ├── maze_burger.launch.py
│       │   └── maze_explore.launch.py
│       ├── worlds/empty.world
│       └── maze_nav/
│           ├── maze.py               # 迷宫生成 + BFS/A* + SDF/JSON/PNG 导出
│           ├── explorer.py           # 未知迷宫探索节点（核心）
│           ├── maze_navigator.py     # 已知地图 A* 导航（对照组）
│           ├── go_to_goal.py         # 里程计 P 控制
│           ├── generate_maze.py      # 命令行生成工具
│           ├── qt_goal_gui.py        # Qt 控制台（PyQt5）
│           └── verify_paths.py       # A* 最短性批量校验
├── docs/
│   ├── algorithm.md                  # 算法推导与代码走读
│   ├── launch.md                     # launch 文件逐段详解
│   ├── troubleshooting.md            # 踩坑记录
│   └── report.docx                   # 课程综合设计报告（含完整实验与分析）
└── assets/                           # README 与文档用图
```

算法细节见 [docs/algorithm.md](docs/algorithm.md)，launch 逐段解析见 [docs/launch.md](docs/launch.md)。

## 常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| `launch configuration 'x' does not exist` | 旧版 `maze_burger.launch.py` 用了未声明的 `x/y/yaw` | **本仓库已修复**；旧版可临时用 `... seed:=42 x:=0.5 y:=0.5 yaw:=0.0` |
| `gzserver ... process has died [exit code 255]` | 上次运行残留的 `gzserver` 占着 11345 端口 | `pkill -f gzserver`（或关掉残留 Gazebo 窗口）后重试 |
| 模型是灰白方块，日志 `Unable to find mesh` | `GAZEBO_MODEL_PATH` 缺 `turtlebot3_gazebo/models` | 用本仓库 launch（会自动补齐）；或手动 `export` |
| Qt 窗口起不来 | 无 `DISPLAY`（纯命令行/WSL 无 WSLg） | 加 `qt_gui:=false` |
| 小车不动 | `maze_burger.launch.py` 需先在 Qt 里点格子给目标 | 点一下格子，或发 `/goal_pose` 话题 |
| 日志反复 `卡住，倒退重规划` | 车轮打滑/顶墙角触发看门狗 | 正常保护逻辑；若频繁出现，降低 `max_linear` |
| 实际速度比指令慢 | `max_linear=0.35` 超过 Burger 标称 0.22 m/s，轮子打滑 | 把 `max_linear` 调到 0.22 以内 |

更多排查过程见 [docs/troubleshooting.md](docs/troubleshooting.md)。

## 开发路线

- [x] 随机 DFS 迷宫生成 + SDF 导出（自包含 world，不依赖 `model://`）
- [x] 未知迷宫探索（贪心 / 随机 DFS + BFS 回溯 + A* 冲刺）
- [x] 四方向雷达分区、占据栅格、状态机与看门狗
- [x] Qt 可视化控制台、已知地图 A* 对照组、P 控制对照组
- [ ] 里程计 + 扫描匹配替代真值位姿（上真机的前提）
- [ ] D\* Lite / 增量式 A* 重规划
- [ ] Pure Pursuit + 速度自适应，替代逐格折线
- [ ] 前沿（frontier）探索 + 信息增益，适配更大迷宫

## 与课程提交版本的差异

为了让仓库开箱即用，本仓库相对课程提交版本做了**一处修正**（不改变算法）：

- `launch/maze_burger.launch.py`：`spawn_entity` 使用了 `LaunchConfiguration('x'/'y'/'yaw')`，
  但 `generate_launch_description()` 未声明这三个参数，导致执行到 spawn 时报
  `launch configuration 'x' does not exist`。现已补上三个 `DeclareLaunchArgument`（默认 `0.5/0.5/0.0`，即起点格心）。

其余代码与报告中描述的行为一致。

## 许可与致谢

- 本项目基于 **Apache License 2.0** 开源，见 [LICENSE](LICENSE)。
- 机器人模型与差速/雷达插件来自 [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)（Apache-2.0），
  仿真器为 [Gazebo](https://classic.gazebosim.org/)，中间件为 [ROS 2 Humble](https://docs.ros.org/en/humble/)。
- 课程：《机器人感知与控制综合实践》（课程代码 (2026-2027-1)-S0610350-01/2/3/4），方向 1「地老鼠走迷宫系统的开发」。
- 报告与实验数据均由本项目代码真实运行得到；报告见 [docs/report.docx](docs/report.docx)。

如果这个项目对你有帮助，欢迎 Star ⭐；有想法或问题请开 Issue。

<div align="center">

Made with ROS 2 + Gazebo · 2026

</div>
