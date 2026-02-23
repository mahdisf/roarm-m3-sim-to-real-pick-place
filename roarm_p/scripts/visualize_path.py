#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def draw_conveyor(ax, center_x, center_y, center_z, width=100, length=300, color="gray"):
    x = np.linspace(center_x - width / 2, center_x + width / 2, 2)
    y = np.linspace(center_y - length / 2, center_y + length / 2, 2)
    x_grid, y_grid = np.meshgrid(x, y)
    z_grid = np.full_like(x_grid, center_z)
    ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=0.3)
    ax.text(center_x, center_y, center_z, "Conveyor", color="black", ha="center")


def visualize_trajectory(json_file: Path, output_file: Path | None = None):
    with json_file.open("r", encoding="utf-8") as file_handle:
        waypoints = json.load(file_handle)

    xs = [point["x"] for point in waypoints]
    ys = [point["y"] for point in waypoints]
    zs = [point["z"] for point in waypoints]

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    draw_conveyor(ax, 250, 0, 0, color="green")
    draw_conveyor(ax, -250, 0, 0, color="red")
    ax.plot(xs, ys, zs, label="Robot Trajectory", color="blue", linewidth=2)

    ax.scatter(250, 0, 15, color="green", s=100, label="Pick")
    ax.scatter(0, 0, 260, color="orange", s=100, label="Arc Peak")
    ax.scatter(-250, 0, 15, color="red", s=100, label="Place")
    ax.scatter(0, 0, 0, color="black", marker="^", s=100, label="Robot Base")

    ax.set_title("Pick -> Arc -> Place Trajectory")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.legend()

    ax.set_xlim(-350, 350)
    ax.set_ylim(-200, 200)
    ax.set_zlim(0, 300)
    ax.set_box_aspect([1, 0.6, 0.5])

    if output_file is not None:
        output_file.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_file, dpi=200, bbox_inches="tight")
    else:
        plt.show()


def default_json_path() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "roarm_p" / "config" / "path_plan.json"


def main():
    parser = argparse.ArgumentParser(description="Visualize planned path from a JSON waypoint file.")
    parser.add_argument("--json", type=Path, default=default_json_path(), help="Path to waypoint JSON file.")
    parser.add_argument("--save", type=Path, default=None, help="If set, save the plot to this path.")
    args = parser.parse_args()
    visualize_trajectory(args.json, args.save)


if __name__ == "__main__":
    main()
