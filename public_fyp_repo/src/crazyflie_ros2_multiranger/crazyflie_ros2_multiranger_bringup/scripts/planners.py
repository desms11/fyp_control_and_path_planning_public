#!/usr/bin/env python3
"""
Custom path planning algorithms for 2D occupancy grids.
Used by navigate_to_pose.py for RRT and GA planners.
"""

import math
import random
import numpy as np


def _bresenham(r0, c0, r1, c1):
    """Integer cells along a line for collision checking."""
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


def _collision_free(grid, a, b):
    """Check if the straight line between two cells is obstacle-free."""
    h, w = grid.shape
    for r, c in _bresenham(a[0], a[1], b[0], b[1]):
        if r < 0 or r >= h or c < 0 or c >= w:
            return False
        if grid[r, c] > 50:
            return False
    return True


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _path_length_cells(path):
    length = 0.0
    for i in range(1, len(path)):
        length += _dist(path[i - 1], path[i])
    return length


def _smooth_path(grid, path, iterations=50):
    """Shortcut smoothing: try to skip intermediate waypoints."""
    if len(path) <= 2:
        return path
    smoothed = list(path)
    for _ in range(iterations):
        if len(smoothed) <= 3:
            break
        i = random.randint(0, len(smoothed) - 3)
        j_max = min(i + 10, len(smoothed) - 1)
        if i + 2 > j_max:
            continue
        j = random.randint(i + 2, j_max)
        if _collision_free(grid, smoothed[i], smoothed[j]):
            smoothed = smoothed[: i + 1] + smoothed[j:]
    return smoothed


def _interpolate_path(path, spacing=1.0):
    """Add intermediate points so no gap exceeds spacing cells."""
    result = [path[0]]
    for i in range(1, len(path)):
        a, b = np.array(path[i - 1], dtype=float), np.array(path[i], dtype=float)
        d = np.linalg.norm(b - a)
        if d > spacing:
            n_pts = int(math.ceil(d / spacing))
            for k in range(1, n_pts):
                t = k / n_pts
                pt = a + t * (b - a)
                result.append((int(round(pt[0])), int(round(pt[1]))))
        result.append(path[i])
    return result


# ──────────────────────────────────────────────
#  RRT Planner
# ──────────────────────────────────────────────

class RRTPlanner:
    def __init__(self, max_iterations=8000, step_size=8, goal_bias=0.10):
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_bias = goal_bias

    def plan(self, grid, start, goal):
        """
        Args:
            grid: 2D numpy array, values >50 are obstacles.
            start: (row, col)
            goal:  (row, col)
        Returns:
            (path, info) where path is list of (row, col) or None.
        """
        h, w = grid.shape
        if grid[start[0], start[1]] > 50 or grid[goal[0], goal[1]] > 50:
            return None, {'iterations': 0, 'tree_size': 0}

        tree = {start: None}
        nodes = [start]

        for it in range(self.max_iterations):
            if random.random() < self.goal_bias:
                sample = goal
            else:
                sample = (random.randint(0, h - 1), random.randint(0, w - 1))

            nearest = min(nodes, key=lambda n: _dist(n, sample))
            direction = np.array(sample, dtype=float) - np.array(nearest, dtype=float)
            d = np.linalg.norm(direction)
            if d < 1e-6:
                continue
            direction /= d
            step = min(self.step_size, d)
            new_pt = np.array(nearest, dtype=float) + direction * step
            new_node = (int(round(new_pt[0])), int(round(new_pt[1])))

            if new_node[0] < 0 or new_node[0] >= h or new_node[1] < 0 or new_node[1] >= w:
                continue
            if grid[new_node[0], new_node[1]] > 50:
                continue
            if not _collision_free(grid, nearest, new_node):
                continue
            if new_node in tree:
                continue

            tree[new_node] = nearest
            nodes.append(new_node)

            if _dist(new_node, goal) <= self.step_size:
                if _collision_free(grid, new_node, goal):
                    tree[goal] = new_node
                    path = []
                    node = goal
                    while node is not None:
                        path.append(node)
                        node = tree[node]
                    path = path[::-1]
                    path = _smooth_path(grid, path)
                    path = _interpolate_path(path, spacing=2.0)
                    return path, {
                        'iterations': it + 1,
                        'tree_size': len(tree),
                    }

        return None, {'iterations': self.max_iterations, 'tree_size': len(tree)}


# ──────────────────────────────────────────────
#  Genetic Algorithm Planner
# ──────────────────────────────────────────────

class GAPlanner:
    def __init__(self, population_size=80, generations=300,
                 mutation_rate=0.15, num_waypoints=6, elite_ratio=0.1):
        self.population_size = population_size
        self.generations = generations
        self.mutation_rate = mutation_rate
        self.num_waypoints = num_waypoints
        self.elite_count = max(2, int(population_size * elite_ratio))

    def _random_individual(self, grid, start, goal):
        """Create a chromosome: list of waypoints between start and goal."""
        h, w = grid.shape
        waypoints = []
        for i in range(self.num_waypoints):
            t = (i + 1) / (self.num_waypoints + 1)
            base_r = start[0] + t * (goal[0] - start[0])
            base_c = start[1] + t * (goal[1] - start[1])
            r = int(np.clip(base_r + random.gauss(0, h * 0.15), 0, h - 1))
            c = int(np.clip(base_c + random.gauss(0, w * 0.15), 0, w - 1))
            waypoints.append((r, c))
        return waypoints

    def _full_path(self, start, waypoints, goal):
        return [start] + list(waypoints) + [goal]

    def _fitness(self, grid, start, waypoints, goal):
        path = self._full_path(start, waypoints, goal)
        length = 0.0
        collision_penalty = 0.0
        h, w = grid.shape
        for i in range(1, len(path)):
            seg_len = _dist(path[i - 1], path[i])
            length += seg_len
            for r, c in _bresenham(path[i - 1][0], path[i - 1][1],
                                   path[i][0], path[i][1]):
                if r < 0 or r >= h or c < 0 or c >= w:
                    collision_penalty += 100.0
                elif grid[r, c] > 50:
                    collision_penalty += 50.0
        cost = length + collision_penalty
        return 1.0 / (cost + 1e-6)

    def _crossover(self, p1, p2):
        child = []
        for i in range(len(p1)):
            if random.random() < 0.5:
                child.append(p1[i])
            else:
                child.append(p2[i])
        return child

    def _mutate(self, individual, grid):
        h, w = grid.shape
        mutated = list(individual)
        for i in range(len(mutated)):
            if random.random() < self.mutation_rate:
                r = int(np.clip(mutated[i][0] + random.gauss(0, h * 0.08), 0, h - 1))
                c = int(np.clip(mutated[i][1] + random.gauss(0, w * 0.08), 0, w - 1))
                mutated[i] = (r, c)
        return mutated

    def _tournament_select(self, population, fitnesses, k=3):
        indices = random.sample(range(len(population)), k)
        best = max(indices, key=lambda i: fitnesses[i])
        return population[best]

    def plan(self, grid, start, goal):
        h, w = grid.shape
        if grid[start[0], start[1]] > 50 or grid[goal[0], goal[1]] > 50:
            return None, {'generations': 0, 'best_fitness': 0}

        population = [self._random_individual(grid, start, goal)
                       for _ in range(self.population_size)]

        best_ever = None
        best_fitness_ever = -1.0

        for gen in range(self.generations):
            fitnesses = [self._fitness(grid, start, ind, goal) for ind in population]

            ranked = sorted(range(len(population)),
                            key=lambda i: fitnesses[i], reverse=True)
            if fitnesses[ranked[0]] > best_fitness_ever:
                best_fitness_ever = fitnesses[ranked[0]]
                best_ever = list(population[ranked[0]])

            elites = [population[ranked[i]] for i in range(self.elite_count)]

            new_pop = list(elites)
            while len(new_pop) < self.population_size:
                p1 = self._tournament_select(population, fitnesses)
                p2 = self._tournament_select(population, fitnesses)
                child = self._crossover(p1, p2)
                child = self._mutate(child, grid)
                new_pop.append(child)
            population = new_pop

        if best_ever is None:
            return None, {'generations': self.generations, 'best_fitness': 0}

        path = self._full_path(start, best_ever, goal)

        has_collision = False
        for i in range(1, len(path)):
            if not _collision_free(grid, path[i - 1], path[i]):
                has_collision = True
                break

        if has_collision:
            return None, {
                'generations': self.generations,
                'best_fitness': best_fitness_ever,
                'note': 'best path still has collisions',
            }

        path = _smooth_path(grid, path)
        path = _interpolate_path(path, spacing=2.0)
        return path, {
            'generations': self.generations,
            'best_fitness': best_fitness_ever,
        }
