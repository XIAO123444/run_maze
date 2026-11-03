# -*- coding: utf-8 -*-
"""generate_maze.py —— 命令行生成迷宫：world(SDF) + 布局(JSON) + 预览(PNG)。

用法：
    ros2 run maze_nav generate_maze --seed 42
    ros2 run maze_nav generate_maze --seed 42 --png
    ros2 run maze_nav generate_maze --seed 7 --cols 10 --rows 10 --out-dir /tmp/maze
"""

import argparse
import os

from maze_nav.maze import Maze


def build_parser():
    p = argparse.ArgumentParser(
        description='生成随机直角迷宫（默认 10x10 格、1 米一格、墙高 1 米）')
    p.add_argument('--seed', type=int, default=0,
                   help='随机种子；同一个 seed 永远生成同一张迷宫')
    p.add_argument('--cols', type=int, default=10, help='x 方向格子数')
    p.add_argument('--rows', type=int, default=10, help='y 方向格子数')
    p.add_argument('--wall-height', type=float, default=1.0, help='墙高（米）')
    p.add_argument('--wall-thickness', type=float, default=0.08, help='墙厚（米）')
    p.add_argument('--loops', type=int, default=0,
                   help='额外打通的内墙数；>0 会产生多条路线（环路）')
    p.add_argument('--start', type=int, nargs=2, default=(0, 0),
                   metavar=('I', 'J'), help='起点格子，默认 0 0')
    p.add_argument('--goal', type=int, nargs=2, default=None,
                   metavar=('I', 'J'), help='终点格子，默认右上角')
    p.add_argument('--out-dir', default='/tmp/maze_nav', help='输出目录')
    p.add_argument('--name', default=None, help='输出文件名前缀')
    p.add_argument('--png', action='store_true', help='额外生成俯视预览图')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    cols, rows = args.cols, args.rows
    start = tuple(args.start)
    goal = tuple(args.goal) if args.goal else (cols - 1, rows - 1)

    for cell, name in ((start, '起点'), (goal, '终点')):
        i, j = cell
        if not (0 <= i < cols and 0 <= j < rows):
            raise SystemExit('%s %s 超出 %dx%d 迷宫范围' % (name, cell, cols, rows))

    maze = Maze(cols, rows, args.seed,
                wall_height=args.wall_height,
                wall_thickness=args.wall_thickness,
                loops=args.loops)

    route = maze.solve(start, goal)
    if not route:
        raise SystemExit('迷宫不连通——完美迷宫不该出现这种情况')

    os.makedirs(args.out_dir, exist_ok=True)
    name = args.name or 'maze_%d_%dx%d' % (args.seed, cols, rows)

    world_path = os.path.join(args.out_dir, name + '.world')
    json_path = os.path.join(args.out_dir, name + '.json')

    with open(world_path, 'w', encoding='utf-8') as f:
        f.write(maze.to_sdf())

    with open(json_path, 'w', encoding='utf-8') as f:
        import json
        json.dump(maze.to_dict(start, goal), f, ensure_ascii=False, indent=1)

    print('迷宫: %d x %d = %d 格, 墙 %d 段, 种子 %d, 环路 %d'
          % (cols, rows, cols * rows, maze.wall_count(), args.seed,
             maze.loops_removed))
    print('起点 %s (世界坐标 %.2f, %.2f) -> 终点 %s (%.2f, %.2f)'
          % (start, *maze.cell_center(start), goal, *maze.cell_center(goal)))
    print('可行路径: %d 个格子, %d 步' % (len(route), len(route) - 1))
    print('world: %s' % world_path)
    print('json : %s' % json_path)

    if args.png:
        png_path = os.path.join(args.out_dir, name + '.png')
        maze.render_png(png_path, route=route)
        print('png  : %s' % png_path)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
