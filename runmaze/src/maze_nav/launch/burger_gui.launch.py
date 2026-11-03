#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""burger_gui.launch.py —— 起 Burger + 控制器 + Qt 控制台。

一条命令：
    ros2 launch maze_nav burger_gui.launch.py

只开 Gazebo 不开 Qt（远程调试）：
    ros2 launch maze_nav burger_gui.launch.py qt_gui:=false

不开 Gazebo 界面（只看 Qt）：
    ros2 launch maze_nav burger_gui.launch.py gazebo_gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_nav')

    burger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'burger.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gazebo_gui'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items())

    controller = Node(
        package='maze_nav',
        executable='go_to_goal',
        name='go_to_goal',
        output='screen')

    gui = Node(
        package='maze_nav',
        executable='qt_goal_gui',
        name='qt_goal_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('qt_gui')))

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='是否开 Gazebo 界面'),
        DeclareLaunchArgument('qt_gui', default_value='true',
                              description='是否开 Qt 控制台'),
        DeclareLaunchArgument('x', default_value='0.0', description='出发点 x'),
        DeclareLaunchArgument('y', default_value='0.0', description='出发点 y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='出发朝向（弧度）'),
        burger,
        controller,
        gui,
    ])
