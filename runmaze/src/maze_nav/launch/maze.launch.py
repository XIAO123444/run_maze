#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""maze.launch.py —— 生成随机迷宫并在 Gazebo 里打开。

启动时按 seed 现场生成 world（写到 /tmp/maze_nav/），所以不用预先 build 世界文件，
同一个 seed 每次都是同一张迷宫，换 seed 就是另一张。

用法：
    ros2 launch maze_nav maze.launch.py                 # 默认 seed=0
    ros2 launch maze_nav maze.launch.py seed:=42        # 固定迷宫
    ros2 launch maze_nav maze.launch.py seed:=-1        # 每次随机，日志里会打印实际 seed
    ros2 launch maze_nav maze.launch.py gui:=false      # 无界面（远程/CI）
"""

import json
import os
import random
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from maze_nav.maze import Maze


def _launch_maze(context):
    seed_text = LaunchConfiguration('seed').perform(context).strip()
    seed = int(seed_text) if seed_text else 0
    if seed < 0:                       # 负数 = 每次随机
        seed = random.randint(0, 2 ** 31 - 1)

    cols = int(LaunchConfiguration('cols').perform(context))
    loops = int(LaunchConfiguration('loops').perform(context))
    rows = int(LaunchConfiguration('rows').perform(context))

    maze = Maze(cols, rows, seed, loops=loops)
    # 生成迷宫
    route = maze.solve((0, 0), (cols - 1, rows - 1))
    if not route:
        raise RuntimeError('迷宫不连通')

    out_dir = os.path.join(tempfile.gettempdir(), 'maze_nav')
    os.makedirs(out_dir, exist_ok=True)
    world_path = os.path.join(out_dir, 'maze_%d.world' % seed)  #迷宫世界的路径
    json_path = os.path.join(out_dir, 'maze_%d.json' % seed)    #迷宫数据的路径

    with open(world_path, 'w', encoding='utf-8') as f:
        f.write(maze.to_sdf())
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(maze.to_dict(), f, ensure_ascii=False, indent=1)

    gazebo_share = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world_path,
            'verbose': LaunchConfiguration('verbose'),
        }.items())
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')))

    return [
        LogInfo(msg='[maze_nav] seed=%d  %dx%d 格  墙 %d 段  路径 %d 步  world=%s'
                % (seed, cols, rows, maze.wall_count(), len(route) - 1, world_path)),
        gzserver,
        gzclient,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='0',
                              description='迷宫种子；-1 表示每次随机'),
        DeclareLaunchArgument('cols', default_value='10', description='x 方向格子数'),
        DeclareLaunchArgument('rows', default_value='10', description='y 方向格子数'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false 时只跑 gzserver'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='gzserver 详细日志'),
        OpaqueFunction(function=_launch_maze),
    ])
