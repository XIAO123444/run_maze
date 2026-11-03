#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""maze_burger.launch.py —— 迷宫 + Burger + A* 导航 + Qt 控制台。

一条命令：
    ros2 launch maze_nav maze_burger.launch.py seed:=42
    ros2 launch maze_nav maze_burger.launch.py seed:=42 gazebo_gui:=false

车 spawn 在起点格 (0,0) 的格心 (0.5, 0.5)，朝向 +x。
Qt 里点格子 → A* 求最短路 → 车沿路径走 → 到点停。
"回到起点并重跑" 按钮会把车瞬移回起点，重新规划并再跑一遍。
"""

import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from maze_nav.maze import Maze


def _setup(context):
    seed = int(LaunchConfiguration('seed').perform(context))
    cols = int(LaunchConfiguration('cols').perform(context))
    loops = int(LaunchConfiguration('loops').perform(context))
    rows = int(LaunchConfiguration('rows').perform(context))

    maze = Maze(cols, rows, seed, loops=loops)
    route = maze.astar((0, 0), (cols - 1, rows - 1))

    out_dir = os.path.join(tempfile.gettempdir(), 'maze_nav')
    os.makedirs(out_dir, exist_ok=True)
    world = os.path.join(out_dir, 'maze_%d.world' % seed)
    with open(world, 'w', encoding='utf-8') as f:
        f.write(maze.to_sdf())
    with open(os.path.join(out_dir, 'maze_%d.json' % seed), 'w', encoding='utf-8') as f:
        json.dump(maze.to_dict(), f, ensure_ascii=False, indent=1)

    # turtlebot3 网格路径
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    model_paths = [os.path.join(tb3_share, 'models'), '/usr/share/gazebo-11/models']
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_paths.append(os.environ['GAZEBO_MODEL_PATH'])
    os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join(model_paths)

    gazebo_share = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('maze_nav')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')))

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', LaunchConfiguration('entity'),
                   '-file', os.path.join(tb3_share, 'models',
                                         'turtlebot3_burger', 'model.sdf'),
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', '0.01', '-Y', LaunchConfiguration('yaw'),
                   '-timeout', '60'],
        output='screen')

    navigator = Node(
        package='maze_nav', executable='maze_navigator', name='maze_navigator',
        output='screen',
        parameters=[{'seed': seed, 'cols': cols, 'rows': rows,
                     'entity': LaunchConfiguration('entity'),
                     'max_linear': ParameterValue(
                         LaunchConfiguration('speed'), value_type=float)}])

    gui = Node(
        package='maze_nav', executable='qt_goal_gui', name='qt_goal_gui',
        output='screen',
        arguments=['--seed', str(seed), '--cols', str(cols), '--rows', str(rows)],
        condition=IfCondition(LaunchConfiguration('qt_gui')))

    return [
        LogInfo(msg='[maze_nav] seed=%d %dx%d，A* 起点->终点 %d 步，world=%s'
                % (seed, cols, rows, len(route) - 1, world)),
        gzserver, gzclient, spawn, navigator, gui,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('cols', default_value='10'),
        DeclareLaunchArgument('rows', default_value='10'),
        DeclareLaunchArgument('loops', default_value='8',
                              description='额外打通的墙数，>0 产生多条路线'),
        DeclareLaunchArgument('entity', default_value='burger'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('qt_gui', default_value='true'),
        # spawn 位置：默认起点格心 (0.5, 0.5)，朝向 +x
        DeclareLaunchArgument('x', default_value='0.5', description='出生点 x（米）'),
        DeclareLaunchArgument('y', default_value='0.5', description='出生点 y（米）'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='出生朝向（弧度）'),
        DeclareLaunchArgument('speed', default_value='0.28', description='最大线速度 m/s'),
        OpaqueFunction(function=_setup),
    ])
