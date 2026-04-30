#!/usr/bin/env python3
"""Generate a maze world for Crazyflie drone simulation in ROS 2 / Gazebo Harmonic.

Outputs:
  - PGM occupancy grid  (200x200, 0.1 m/cell)
  - ROS map_server YAML metadata
  - Gazebo SDF world with merged wall box models
"""

import os
import random
from collections import deque

SEED = 42
MAP_W, MAP_H = 200, 200
RES = 0.1
ORIGIN_X, ORIGIN_Y = -10.0, -10.0

ROWS, COLS = 6, 8
CS = 10   # cell pitch in pixels
WT = 2    # wall thickness in pixels
FREE, OCC = 254, 0
WALL_H = 0.5

CS_M = CS * RES   # 1.0 m
WT_M = WT * RES   # 0.2 m

MAZE_W_PX = COLS * CS + WT  # 82
MAZE_H_PX = ROWS * CS + WT  # 62
MS_C = (MAP_W - MAZE_W_PX) // 2  # 59
MS_R = (MAP_H - MAZE_H_PX) // 2  # 69

# World-coordinate bounds of the maze region
X_MIN = ORIGIN_X + MS_C * RES          # -4.1
Y_MAX = ORIGIN_Y + (MAP_H - MS_R) * RES  # 3.1

PGM_PATH = "/home/ubunuser/ros2_ws/maps/maze_map.pgm"
YAML_PATH = "/home/ubunuser/ros2_ws/maps/maze_map.yaml"
SDF_PATH = (
    "/home/ubunuser/ros2_ws/src/ros_gz_crazyflie/"
    "ros_gz_crazyflie_gazebo/worlds/maze_world.sdf"
)


# ---------------------------------------------------------------------------
# Maze generation (recursive backtracker / DFS)
# ---------------------------------------------------------------------------

def generate_maze():
    random.seed(SEED)
    cells = [
        [{"N": True, "S": True, "E": True, "W": True} for _ in range(COLS)]
        for _ in range(ROWS)
    ]
    visited = [[False] * COLS for _ in range(ROWS)]

    stack = [(ROWS - 1, 0)]
    visited[ROWS - 1][0] = True

    directions = [
        (-1, 0, "N", "S"),
        (1, 0, "S", "N"),
        (0, 1, "E", "W"),
        (0, -1, "W", "E"),
    ]

    while stack:
        r, c = stack[-1]
        nbrs = []
        for dr, dc, wall, opp in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < ROWS and 0 <= nc < COLS and not visited[nr][nc]:
                nbrs.append((nr, nc, wall, opp))
        if nbrs:
            nr, nc, wall, opp = random.choice(nbrs)
            cells[r][c][wall] = False
            cells[nr][nc][opp] = False
            visited[nr][nc] = True
            stack.append((nr, nc))
        else:
            stack.pop()

    # Entrance (south of bottom-left cell) and exit (north of top-right cell)
    cells[ROWS - 1][0]["S"] = False
    cells[0][COLS - 1]["N"] = False
    return cells


# ---------------------------------------------------------------------------
# PGM pixel rendering
# ---------------------------------------------------------------------------

def render_pixels(cells):
    grid = [[FREE] * MAP_W for _ in range(MAP_H)]

    # Corner posts at every grid intersection (always solid)
    for r in range(ROWS + 1):
        for c in range(COLS + 1):
            pr, pc = MS_R + r * CS, MS_C + c * CS
            for dr in range(WT):
                for dc in range(WT):
                    grid[pr + dr][pc + dc] = OCC

    # North walls (horizontal between corner posts)
    for r in range(ROWS):
        for c in range(COLS):
            if cells[r][c]["N"]:
                pr = MS_R + r * CS
                pc = MS_C + c * CS + WT
                for dr in range(WT):
                    for dc in range(CS - WT):
                        grid[pr + dr][pc + dc] = OCC

    # South boundary walls
    for c in range(COLS):
        if cells[ROWS - 1][c]["S"]:
            pr = MS_R + ROWS * CS
            pc = MS_C + c * CS + WT
            for dr in range(WT):
                for dc in range(CS - WT):
                    grid[pr + dr][pc + dc] = OCC

    # West walls (vertical between corner posts)
    for r in range(ROWS):
        for c in range(COLS):
            if cells[r][c]["W"]:
                pr = MS_R + r * CS + WT
                pc = MS_C + c * CS
                for dr in range(CS - WT):
                    for dc in range(WT):
                        grid[pr + dr][pc + dc] = OCC

    # East boundary walls
    for r in range(ROWS):
        if cells[r][COLS - 1]["E"]:
            pr = MS_R + r * CS + WT
            pc = MS_C + COLS * CS
            for dr in range(CS - WT):
                for dc in range(WT):
                    grid[pr + dr][pc + dc] = OCC

    return grid


# ---------------------------------------------------------------------------
# File writers
# ---------------------------------------------------------------------------

def write_pgm(grid):
    os.makedirs(os.path.dirname(PGM_PATH), exist_ok=True)
    with open(PGM_PATH, "wb") as f:
        f.write(f"P5\n{MAP_W} {MAP_H}\n{FREE}\n".encode())
        for row in grid:
            f.write(bytes(row))


def write_yaml():
    os.makedirs(os.path.dirname(YAML_PATH), exist_ok=True)
    with open(YAML_PATH, "w") as f:
        f.write(
            "image: maze_map.pgm\n"
            "mode: trinary\n"
            "resolution: 0.100\n"
            "origin: [-10.000, -10.000, 0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )


# ---------------------------------------------------------------------------
# SDF wall generation from cell structure (with merging)
# ---------------------------------------------------------------------------

def build_sdf_walls(cells):
    """Return list of (idx, cx, cy, size_x, size_y) tuples.

    Horizontal walls include the right-hand corner post (+WT_M extension)
    so that every grid intersection is covered by at least one wall model.
    Vertical walls include a bottom corner post extension for the same reason.
    """
    walls = []
    idx = 0

    # --- Horizontal walls (row boundaries 0..ROWS) ---
    for r in range(ROWS + 1):
        segs = []
        for c in range(COLS):
            has = cells[r][c]["N"] if r < ROWS else cells[ROWS - 1][c]["S"]
            if has:
                if segs and segs[-1][1] == c - 1:
                    segs[-1] = (segs[-1][0], c)
                else:
                    segs.append((c, c))
        for c0, c1 in segs:
            n = c1 - c0 + 1
            length = n * CS_M + WT_M
            cx = X_MIN + c0 * CS_M + length / 2.0
            cy = Y_MAX - r * CS_M - WT_M / 2.0
            walls.append((idx, cx, cy, length, WT_M))
            idx += 1

    # --- Vertical walls (column boundaries 0..COLS) ---
    for c in range(COLS + 1):
        segs = []
        for r in range(ROWS):
            has = cells[r][c]["W"] if c < COLS else cells[r][COLS - 1]["E"]
            if has:
                if segs and segs[-1][1] == r - 1:
                    segs[-1] = (segs[-1][0], r)
                else:
                    segs.append((r, r))
        for r0, r1 in segs:
            n = r1 - r0 + 1
            height = n * CS_M + WT_M
            cx = X_MIN + c * CS_M + WT_M / 2.0
            cy = Y_MAX - r0 * CS_M - height / 2.0
            walls.append((idx, cx, cy, WT_M, height))
            idx += 1

    return walls


def write_sdf(walls):
    os.makedirs(os.path.dirname(SDF_PATH), exist_ok=True)

    wall_models = []
    for i, cx, cy, sx, sy in walls:
        wall_models.append(
            f"    <model name='wall_{i}'>\n"
            f"      <static>1</static>\n"
            f"      <pose>{cx:.3f} {cy:.3f} 0.25 0 0 0</pose>\n"
            f"      <link name='link'>\n"
            f"        <collision name='collision'>"
            f"<geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size>"
            f"</box></geometry></collision>\n"
            f"        <visual name='visual'>"
            f"<geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size>"
            f"</box></geometry>\n"
            f"          <material><ambient>0.5 0.5 0.5 1</ambient></material>\n"
            f"        </visual>\n"
            f"      </link>\n"
            f"    </model>"
        )

    wall_xml = "\n".join(wall_models)

    sdf = (
        '<?xml version="1.0" ?>\n'
        "<sdf version='1.6'>\n"
        "  <world name='default'>\n"
        "    <plugin filename='gz-sim-physics-system'"
        " name='gz::sim::systems::Physics'/>\n"
        "    <plugin filename='gz-sim-sensors-system'"
        " name='gz::sim::systems::Sensors'>\n"
        "      <render_engine>ogre2</render_engine>\n"
        "    </plugin>\n"
        "    <plugin filename='gz-sim-scene-broadcaster-system'"
        " name='gz::sim::systems::SceneBroadcaster'/>\n"
        "    <plugin filename='gz-sim-user-commands-system'"
        " name='gz::sim::systems::UserCommands'/>\n"
        "    <light name='sun' type='directional'>\n"
        "      <cast_shadows>1</cast_shadows>\n"
        "      <pose>0 0 10 0 0 0</pose>\n"
        "      <diffuse>0.8 0.8 0.8 1</diffuse>\n"
        "      <specular>0.2 0.2 0.2 1</specular>\n"
        "      <attenuation><range>1000</range><constant>0.9</constant>"
        "<linear>0.01</linear><quadratic>0.001</quadratic></attenuation>\n"
        "      <direction>-0.5 0.1 -0.9</direction>\n"
        "    </light>\n"
        "    <model name='ground_plane'>\n"
        "      <static>1</static>\n"
        "      <link name='link'>\n"
        "        <collision name='collision'><geometry><plane>"
        "<normal>0 0 1</normal><size>100 100</size>"
        "</plane></geometry></collision>\n"
        "        <visual name='visual'><cast_shadows>0</cast_shadows>"
        "<geometry><plane><normal>0 0 1</normal>"
        "<size>100 100</size></plane></geometry></visual>\n"
        "      </link>\n"
        "    </model>\n"
        "    <include>\n"
        "      <uri>model://crazyflie</uri>\n"
        "      <name>crazyflie</name>\n"
        "      <pose>0 0 0 0 0 0</pose>\n"
        "    </include>\n"
        "    <gravity>0 0 -9.8</gravity>\n"
        "    <physics name='default_physics' default='0' type='ode'>\n"
        "      <max_step_size>0.001</max_step_size>\n"
        "      <real_time_factor>1</real_time_factor>\n"
        "      <real_time_update_rate>1000</real_time_update_rate>\n"
        "    </physics>\n"
        f"{wall_xml}\n"
        "  </world>\n"
        "</sdf>\n"
    )

    with open(SDF_PATH, "w") as f:
        f.write(sdf)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def shortest_path(cells):
    """BFS from entrance (bottom-left) to exit (top-right)."""
    start, goal = (ROWS - 1, 0), (0, COLS - 1)
    queue = deque([(start, [start])])
    seen = {start}
    directions = [(-1, 0, "N"), (1, 0, "S"), (0, 1, "E"), (0, -1, "W")]

    while queue:
        (r, c), path = queue.popleft()
        if (r, c) == goal:
            return path
        for dr, dc, wall in directions:
            nr, nc = r + dr, c + dc
            if (
                0 <= nr < ROWS
                and 0 <= nc < COLS
                and (nr, nc) not in seen
                and not cells[r][c][wall]
            ):
                seen.add((nr, nc))
                queue.append(((nr, nc), path + [(nr, nc)]))
    return None


def print_maze(cells):
    for r in range(ROWS):
        top = ""
        for c in range(COLS):
            top += "+" + ("--" if cells[r][c]["N"] else "  ")
        top += "+"
        print(top)

        mid = ""
        for c in range(COLS):
            mid += ("|" if cells[r][c]["W"] else " ") + "  "
        mid += "|" if cells[r][COLS - 1]["E"] else " "
        print(mid)

    bot = ""
    for c in range(COLS):
        bot += "+" + ("--" if cells[ROWS - 1][c]["S"] else "  ")
    bot += "+"
    print(bot)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print(f"Generating maze (seed={SEED}, {COLS}x{ROWS} cells)...")
    cells = generate_maze()

    print(f"\nMaze layout ({COLS}x{ROWS} cells):")
    print_maze(cells)

    path = shortest_path(cells)
    if path:
        dist_m = (len(path) - 1) * CS_M
        time_s = dist_m / 0.65
        print(
            f"\nShortest path: {len(path)} cells, "
            f"~{dist_m:.1f} m, ~{time_s:.1f} s at 0.65 m/s"
        )
        assert time_s < 120, "Path too long for 2-minute constraint!"
    else:
        raise RuntimeError("No path from entrance to exit!")

    print(f"\nRendering PGM ({MAP_W}x{MAP_H} pixels, {RES} m/cell)...")
    grid = render_pixels(cells)
    write_pgm(grid)
    print(f"  {PGM_PATH}")

    write_yaml()
    print(f"  {YAML_PATH}")

    print("\nBuilding SDF walls (cell-based, merged)...")
    sdf_walls = build_sdf_walls(cells)
    print(f"  {len(sdf_walls)} wall models")
    write_sdf(sdf_walls)
    print(f"  {SDF_PATH}")

    print("\nDone — all files generated successfully.")


if __name__ == "__main__":
    main()
