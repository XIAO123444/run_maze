#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""burger_goal.launch.py —— 起 Burger + 让它开到指定 (x, y)。

一条命令：
    ros2 launch maze_nav burger_goal.launch.py goal_x:=3.0 goal_y:=2.0
    ros2 launch maze_nav burger_goal.launch.py goal_x:=3.0 goal_y:=2.0 gui:=false

车默认从 (0,0) 出发；想换出发点用 x:= y:= yaw:=。
到达后会打印"到达目标"，车停下；再发 /goal_pose 可以去下一个点。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_nav')

    burger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'burger.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items())

    goal_node = Node(
        package='maze_nav',
        executable='go_to_goal',
        name='go_to_goal',
        output='screen',
        parameters=[{
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='1.0', description='目标 x（米）'),
        DeclareLaunchArgument('goal_y', default_value='1.0', description='目标 y（米）'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0', description='出发点 x'),
        DeclareLaunchArgument('y', default_value='0.0', description='出发点 y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='出发朝向（弧度）'),
        burger,
        goal_node,
    ])
