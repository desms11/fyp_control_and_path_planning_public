#!/usr/bin/env python3
"""
Visualize the navigation map with world-coordinate grid overlay.
Click on the map to print world coordinates you can pass as goal_x / goal_y.

Usage:
  python3 ~/ros2_ws/scripts/show_map_coords.py
  python3 ~/ros2_ws/scripts/show_map_coords.py --map ~/ros2_ws/maps/cf_room2_map.yaml
"""

import argparse
import os
import yaml
import numpy as np

try:
    from PIL import Image
except ImportError:
    Image = None

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    map_dir = os.path.dirname(os.path.abspath(yaml_path))
    img_path = os.path.join(map_dir, meta['image'])
    resolution = float(meta['resolution'])
    origin = meta['origin']
    ox, oy = float(origin[0]), float(origin[1])

    if Image is not None:
        img = np.array(Image.open(img_path))
    else:
        import cv2
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

    return img, resolution, ox, oy


def print_map_info(img, resolution, ox, oy):
    h, w = img.shape[:2]
    max_x = ox + w * resolution
    max_y = oy + h * resolution
    print(f"Map image:      {w} x {h} pixels")
    print(f"Resolution:     {resolution} m/pixel")
    print(f"Origin (lower-left): ({ox}, {oy})")
    print(f"X range:        {ox:.1f}  to  {max_x:.1f} m")
    print(f"Y range:        {oy:.1f}  to  {max_y:.1f} m")
    print(f"Center:         ({(ox + max_x)/2:.1f}, {(oy + max_y)/2:.1f})")
    print()

    free_count = np.sum(img > 200) if img.ndim == 2 else np.sum(img[:,:,0] > 200)
    wall_count = np.sum(img < 50) if img.ndim == 2 else np.sum(img[:,:,0] < 50)
    print(f"Free cells:     {free_count}")
    print(f"Wall cells:     {wall_count}")
    print()
    print("--- Some example free-space coordinates ---")
    if img.ndim == 3:
        gray = img[:,:,0]
    else:
        gray = img
    free_pixels = np.argwhere(gray > 200)
    if len(free_pixels) > 0:
        indices = np.linspace(0, len(free_pixels)-1, min(8, len(free_pixels)), dtype=int)
        for idx in indices:
            py, px = free_pixels[idx]
            world_x = ox + px * resolution
            world_y = oy + (img.shape[0] - 1 - py) * resolution
            print(f"  pixel ({px:4d}, {py:4d})  ->  world ({world_x:6.1f}, {world_y:6.1f})")


def show_interactive(img, resolution, ox, oy):
    if not HAS_MPL:
        print("\nmatplotlib not available — install it for interactive clicking:")
        print("  pip3 install matplotlib")
        return

    h, w = img.shape[:2]
    extent = [ox, ox + w * resolution, oy, oy + h * resolution]

    # PGM row 0 = top of map (highest Y). Flip so row 0 = bottom (lowest Y)
    # to match matplotlib's origin='lower' convention.
    display_img = np.flipud(img)

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.imshow(display_img, cmap='gray', origin='lower', extent=extent)
    ax.set_xlabel('X (meters) — use as goal_x')
    ax.set_ylabel('Y (meters) — use as goal_y')
    ax.set_title('Click on free space (white) to get goal coordinates\nClose window when done')
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.set_aspect('equal')

    major_step = 1.0
    ax.set_xticks(np.arange(int(extent[0]), int(extent[1])+1, major_step))
    ax.set_yticks(np.arange(int(extent[2]), int(extent[3])+1, major_step))

    markers = []

    def on_click(event):
        if event.inaxes != ax:
            return
        x, y = event.xdata, event.ydata
        px = int((x - ox) / resolution)
        py = int((y - oy) / resolution)
        py_img = h - 1 - py
        if 0 <= px < w and 0 <= py_img < h:
            gray_val = img[py_img, px] if img.ndim == 2 else img[py_img, px, 0]
            status = "FREE" if gray_val > 200 else ("WALL" if gray_val < 50 else "UNKNOWN")
        else:
            status = "OUT OF BOUNDS"

        print(f"\n  Clicked: world ({x:.2f}, {y:.2f})  [{status}]")
        if status == "FREE":
            print(f"  Use as: -p goal_x:={x:.2f} -p goal_y:={y:.2f}")

        dot, = ax.plot(x, y, 'rx' if status != "FREE" else 'go', markersize=10, markeredgewidth=2)
        markers.append(dot)
        fig.canvas.draw()

    fig.canvas.mpl_connect('button_press_event', on_click)
    print("\nInteractive map opened. Click on white (free) areas to get coordinates.")
    print("Close the window when done.\n")
    plt.show()


def main():
    default_map = os.path.expanduser('~/ros2_ws/maps/cf_room2_map.yaml')
    parser = argparse.ArgumentParser(description='View map coordinates')
    parser.add_argument('--map', default=default_map, help='Path to map .yaml file')
    parser.add_argument('--no-gui', action='store_true', help='Print info only, no interactive window')
    args = parser.parse_args()

    img, resolution, ox, oy = load_map(args.map)
    print_map_info(img, resolution, ox, oy)

    if not args.no_gui:
        show_interactive(img, resolution, ox, oy)


if __name__ == '__main__':
    main()
