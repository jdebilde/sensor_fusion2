#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt


COLUMNS = ["seq", "timestamp", "x", "y", "vx", "vy"]


def load_ekf_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, header=None, names=COLUMNS)

    for col in COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    return df.dropna(subset=["x", "y"])


def parse_truth_from_filename(path: Path):
    name = path.stem

    static_match = re.search(r"static_([0-9]+)_([0-9]+)", name)
    if static_match:
        x_cm = float(static_match.group(1))
        y_cm = float(static_match.group(2))
        return "static", [(x_cm / 100.0, y_cm / 100.0)]

    moving_match = re.search(r"moving_([0-9_]+(?:__[0-9_]+)*)", name)
    if moving_match:
        raw_points = moving_match.group(1).split("__")

        points = []
        for raw in raw_points:
            x_cm, y_cm = raw.split("_")
            points.append((float(x_cm) / 100.0, float(y_cm) / 100.0))

        return "moving", points

    return "unknown", []


def plot_ekf_file(path: Path, show_velocity: bool = False):
    df = load_ekf_csv(path)
    truth_type, truth_points = parse_truth_from_filename(path)

    plt.figure(figsize=(8, 7))

    # EKF trajectory
    plt.plot(df["x"], df["y"], label="EKF estimated position")
    plt.scatter(df["x"].iloc[0], df["y"].iloc[0], marker="o", label="EKF start")
    plt.scatter(df["x"].iloc[-1], df["y"].iloc[-1], marker="x", label="EKF end")

    # Optional velocity arrows
    if show_velocity:
        step = max(1, len(df) // 40)
        plt.quiver(
            df["x"].iloc[::step],
            df["y"].iloc[::step],
            df["vx"].iloc[::step],
            df["vy"].iloc[::step],
            angles="xy",
            scale_units="xy",
            scale=1,
            width=0.003,
            label="Velocity",
        )

    # Ground truth
    if truth_type == "static":
        x_true, y_true = truth_points[0]
        plt.scatter(
            [x_true],
            [y_true],
            marker="*",
            s=200,
            label=f"True position ({x_true:.2f}, {y_true:.2f})",
        )

    elif truth_type == "moving":
        xs = [p[0] for p in truth_points]
        ys = [p[1] for p in truth_points]

        plt.plot(xs, ys, "--o", label="True trajectory")

        for i, (x, y) in enumerate(truth_points):
            plt.text(x, y, f" P{i+1}")

    else:
        print(f"Could not parse true position from filename: {path.name}")

    plt.title(path.name)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="EKF CSV file to plot")
    parser.add_argument(
        "--velocity",
        action="store_true",
        help="Show velocity arrows",
    )

    args = parser.parse_args()
    plot_ekf_file(Path(args.file), show_velocity=args.velocity)


if __name__ == "__main__":
    main()