#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""maze_explore.launch.py —— 未知迷宫自主探索 + A*。

和 maze_burger 的区别：不给车迷宫地图。车只有 /scan 和终点坐标，
自己建图、自己找路；终点通路一旦被发现，就用 A* 过去。

    ros2 launch maze_nav maze_explore.launch.py seed:=3
    ros2 launch maze_nav maze_explore.launch.py seed:=3 cols:=6 rows:=6
    ros2 launch maze_nav maze_explore.launch.py seed:=3 gazebo_gui:=false
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

from maze_nav.maze import Maze


def _setup(context):
    seed = int(LaunchConfiguration('seed').perform(context))
    cols = int(LaunchConfiguration('cols').perform(context))
    loops = int(LaunchConfiguration('loops').perform(context))
    rows = int(LaunchConfiguration('rows').perform(context))

    maze = Maze(cols, rows, seed, loops=loops)
    out_dir = os.path.join(tempfile.gettempdir(), 'maze_nav')
    os.makedirs(out_dir, exist_ok=True)
    world = os.path.join(out_dir, 'maze_%d_%dx%d.world' % (seed, cols, rows))
    with open(world, 'w', encoding='utf-8') as f:
        f.write(maze.to_sdf())
    with open(os.path.join(out_dir, 'maze_%d.json' % seed), 'w', encoding='utf-8') as f:
        json.dump(maze.to_dict(), f, ensure_ascii=False, indent=1)

    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    model_paths = [os.path.join(tb3_share, 'models'), '/usr/share/gazebo-11/models']
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_paths.append(os.environ['GAZEBO_MODEL_PATH'])
    os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join(model_paths)

    gazebo_share = get_package_share_directory('gazebo_ros')

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
                   '-x', '0.5', '-y', '0.5', '-z', '0.01', '-Y', '0.0',
                   '-timeout', '60'],
        output='screen')

    explorer = Node(
        package='maze_nav', executable='maze_explorer', name='maze_explorer',
        output='screen',
        parameters=[{
            'goal_x': float(cols - 0.5),
            'goal_y': float(rows - 0.5),
            'cols': cols,
            'rows': rows,
            'entity': LaunchConfiguration('entity'),
            'strategy': LaunchConfiguration('strategy'),
            'explore_all': LaunchConfiguration('explore_all'),
        }])

    gui = Node(
        package='maze_nav', executable='qt_goal_gui', name='qt_goal_gui',
        output='screen',
        arguments=['--explore', '--goal-x', str(cols - 0.5),
                   '--goal-y', str(rows - 0.5),
                   '--xmin', '-1', '--ymin', '-1',
                   '--xmax', str(cols + 1), '--ymax', str(rows + 1)],
        condition=IfCondition(LaunchConfiguration('qt_gui')))

    return [
        LogInfo(msg='[maze_nav] 未知迷宫探索 seed=%d %dx%d，终点 (%.1f, %.1f)'
                % (seed, cols, rows, cols - 0.5, rows - 0.5)),
        gzserver, gzclient, spawn, explorer, gui,
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
        DeclareLaunchArgument('strategy', default_value='random',
                              description='random=DFS 乱逛；greedy=朝终点'),
        DeclareLaunchArgument('explore_all', default_value='false',
                              description='true=先逛完整座迷宫再 A* 去终点（回溯多）'),
        OpaqueFunction(function=_setup),
    ])
