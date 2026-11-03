import os
from glob import glob

from setuptools import setup

package_name = 'maze_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moyou',
    maintainer_email='moyou@todo.todo',
    description='10x10 随机迷宫 + Gazebo 仿真：迷宫生成、点到点导航、A* 验证',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_maze = maze_nav.generate_maze:main',
            'go_to_goal = maze_nav.go_to_goal:main',
            'qt_goal_gui = maze_nav.qt_goal_gui:main',
            'maze_navigator = maze_nav.maze_navigator:main',
            'verify_paths = maze_nav.verify_paths:main',
            'maze_explorer = maze_nav.explorer:main',
        ],
    },
)
