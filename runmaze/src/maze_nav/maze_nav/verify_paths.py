# -*- coding: utf-8 -*-
"""verify_paths.py —— 验证 A* 的最短性与路径合法性。

对每个种子：
    1) A* 路径长度 == BFS 路径长度（网格单位代价下 A* 必须最优）；
    2) A* 路径每一步都是相邻格子，且中间没有墙；
    3) 起点/终点正确。
另外随机抽若干对格子重复上面检查。

用法：
    ros2 run maze_nav verify_paths
    ros2 run maze_nav verify_paths --seeds 200
"""

import argparse
import random

from maze_nav.maze import Maze


def check_pair(maze, start, goal):
    a = maze.astar(start, goal)
    b = maze.solve(start, goal)
    if a is None or b is None:
        return 'no path'
    if len(a) != len(b):
        return 'A*=%d BFS=%d' % (len(a), len(b))
    if a[0] != start or a[-1] != goal:
        return 'endpoints wrong'
    for u, v in zip(a, a[1:]):
        if v not in maze.neighbors(u):
            return 'step %s->%s crosses a wall' % (u, v)
    return None


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--seeds', type=int, default=100)
    p.add_argument('--cols', type=int, default=10)
    p.add_argument('--rows', type=int, default=10)
    p.add_argument('--random-pairs', type=int, default=20)
    p.add_argument('--loops', type=int, default=0, help='额外打通的墙数（生成环路）')
    args = p.parse_args(argv)

    problems = []
    total_steps = 0
    for seed in range(args.seeds):
        m = Maze(args.cols, args.rows, seed, loops=args.loops)
        err = check_pair(m, (0, 0), (args.cols - 1, args.rows - 1))
        if err:
            problems.append((seed, 'corner', err))
        total_steps += len(m.astar((0, 0), (args.cols - 1, args.rows - 1))) - 1

        rng = random.Random(seed * 7919)
        for _ in range(args.random_pairs):
            s = (rng.randrange(args.cols), rng.randrange(args.rows))
            g = (rng.randrange(args.cols), rng.randrange(args.rows))
            err = check_pair(m, s, g)
            if err:
                problems.append((seed, (s, g), err))

    print('检查 %d 个种子 x %d 对格子（环路 %d）'
          % (args.seeds, args.random_pairs + 1, args.loops))
    print('平均最短路（对角）: %.1f 步' % (total_steps / args.seeds))
    if problems:
        print('发现 %d 个问题:' % len(problems))
        for item in problems[:10]:
            print(' ', item)
        return 1
    print('全部通过：A* 路径长度与 BFS 一致，且没有穿墙。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
