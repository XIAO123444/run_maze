#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""burger.launch.py —— 只启动 TurtleBot3 Burger，不加载迷宫。

用来单独验证小车本身：
    · Gazebo 空场地（自包含世界，不引用 model://ground_plane）
    · 从 turtlebot3_gazebo 的 model.sdf spawn（自带差速驱动/雷达/IMU/关节状态）
    · 话题：/cmd_vel  /odom  /scan  /imu  /joint_states

用法：
    ros2 launch maze_nav burger.launch.py
    ros2 launch maze_nav burger.launch.py gui:=false
    ros2 launch maze_nav burger.launch.py x:=1.0 y:=2.0 yaw:=1.57

开车：
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.8}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_nav')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_share = get_package_share_directory('gazebo_ros')

    # turtlebot3 的 model.sdf 用 model://turtlebot3_common/... 引用网格，
    # 必须保证模型路径里有 turtlebot3_gazebo/models。
    # 这里在生成 launch 描述时就把环境补齐（gzserver.launch.py 会继承）。
    model_paths = [os.path.join(tb3_share, 'models'), '/usr/share/gazebo-11/models']
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_paths.append(os.environ['GAZEBO_MODEL_PATH'])
    os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join(model_paths)

    world = os.path.join(pkg_share, 'worlds', 'empty.world')
    burger_sdf = os.path.join(tb3_share, 'models', 'turtlebot3_burger', 'model.sdf')

    arg_gui = DeclareLaunchArgument('gui', default_value='true',
                                    description='是否打开 Gazebo 界面')
    arg_verbose = DeclareLaunchArgument('verbose', default_value='false')
    arg_entity = DeclareLaunchArgument('entity', default_value='burger')
    arg_x = DeclareLaunchArgument('x', default_value='0.0')
    arg_y = DeclareLaunchArgument('y', default_value='0.0')
    arg_z = DeclareLaunchArgument('z', default_value='0.01')
    arg_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world,
                          'verbose': LaunchConfiguration('verbose')}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')))

    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', burger_sdf,
                       '-entity', LaunchConfiguration('entity'),
                       '-x', LaunchConfiguration('x'),
                       '-y', LaunchConfiguration('y'),
                       '-z', LaunchConfiguration('z'),
                       '-Y', LaunchConfiguration('yaw'),
                       '-timeout', '60'],
            output='screen')])

    return LaunchDescription([
        arg_gui, arg_verbose, arg_entity, arg_x, arg_y, arg_z, arg_yaw,
        gzserver, gzclient, spawn,
    ])
