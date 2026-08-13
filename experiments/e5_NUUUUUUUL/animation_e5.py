#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


COLUMNS = ["seq", "timestamp", "x", "y", "vx", "vy"]


def load_ekf_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, header=None, names=COLUMNS)

    for col in COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["x", "y", "vx", "vy"]).reset_index(drop=True)

    if df.empty:
        raise ValueError(f"No valid EKF samples found in {path}")

    return df


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


def compute_plot_limits(df: pd.DataFrame, truth_points):
    xs = list(df["x"])
    ys = list(df["y"])

    for x, y in truth_points:
        xs.append(x)
        ys.append(y)

    margin = 0.25
    xmin, xmax = min(xs) - margin, max(xs) + margin
    ymin, ymax = min(ys) - margin, max(ys) + margin

    # Avoid zero-size axes
    if abs(xmax - xmin) < 0.5:
        center = (xmin + xmax) / 2
        xmin, xmax = center - 0.25, center + 0.25

    if abs(ymax - ymin) < 0.5:
        center = (ymin + ymax) / 2
        ymin, ymax = center - 0.25, center + 0.25

    return xmin, xmax, ymin, ymax


def animate_ekf_file(
    path: Path,
    interval_ms: int = 40,
    trail: int | None = None,
    save_path: Path | None = None,
):
    df = load_ekf_csv(path)
    truth_type, truth_points = parse_truth_from_filename(path)

    fig, ax = plt.subplots(figsize=(8, 7))

    xmin, xmax, ymin, ymax = compute_plot_limits(df, truth_points)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    ax.set_title(path.name)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    # Ground truth
    if truth_type == "static":
        x_true, y_true = truth_points[0]
        ax.scatter(
            [x_true],
            [y_true],
            marker="*",
            s=200,
            label=f"True position ({x_true:.2f}, {y_true:.2f})",
        )

    elif truth_type == "moving":
        xs = [p[0] for p in truth_points]
        ys = [p[1] for p in truth_points]
        ax.plot(xs, ys, "--o", label="True trajectory")

        for i, (x, y) in enumerate(truth_points):
            ax.text(x, y, f" P{i+1}")

    else:
        print(f"Could not parse true trajectory from filename: {path.name}")

    # EKF animated objects
    estimated_line, = ax.plot([], [], label="EKF trajectory")
    current_point, = ax.plot([], [], marker="o", linestyle="None", label="Current EKF position")

    # Velocity arrow. We update it with annotate because it is simple and readable.
    velocity_arrow = ax.annotate(
        "",
        xy=(0, 0),
        xytext=(0, 0),
        arrowprops=dict(arrowstyle="->", linewidth=2),
    )

    info_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
    )

    ax.legend(loc="lower right")

    def get_window(i):
        if trail is None:
            start = 0
        else:
            start = max(0, i - trail + 1)
        end = i + 1
        return start, end

    def init():
        estimated_line.set_data([], [])
        current_point.set_data([], [])
        velocity_arrow.xy = (0, 0)
        velocity_arrow.set_position((0, 0))
        info_text.set_text("")
        return estimated_line, current_point, velocity_arrow, info_text

    def update(i):
        start, end = get_window(i)

        x_values = df["x"].iloc[start:end]
        y_values = df["y"].iloc[start:end]

        x = df["x"].iloc[i]
        y = df["y"].iloc[i]
        vx = df["vx"].iloc[i]
        vy = df["vy"].iloc[i]

        speed = (vx**2 + vy**2) ** 0.5

        estimated_line.set_data(x_values, y_values)
        current_point.set_data([x], [y])

        # Scale arrow so it remains visible. This is visual only.
        arrow_scale = 0.25
        velocity_arrow.xy = (x + vx * arrow_scale, y + vy * arrow_scale)
        velocity_arrow.set_position((x, y))

        info_text.set_text(
            f"sample: {i + 1}/{len(df)}\n"
            f"seq: {int(df['seq'].iloc[i]) if pd.notna(df['seq'].iloc[i]) else 'NA'}\n"
            f"x: {x:.3f} m\n"
            f"y: {y:.3f} m\n"
            f"vx: {vx:.3f} m/s\n"
            f"vy: {vy:.3f} m/s\n"
            f"|v|: {speed:.3f} m/s"
        )

        return estimated_line, current_point, velocity_arrow, info_text

    anim = FuncAnimation(
        fig,
        update,
        frames=len(df),
        init_func=init,
        interval=interval_ms,
        blit=False,
        repeat=False,
    )

    if save_path is not None:
        suffix = save_path.suffix.lower()
        if suffix == ".gif":
            anim.save(save_path, writer="pillow", fps=max(1, int(1000 / interval_ms)))
        elif suffix in {".mp4", ".m4v"}:
            anim.save(save_path, writer="ffmpeg", fps=max(1, int(1000 / interval_ms)))
        else:
            raise ValueError("save path must end with .gif or .mp4")

        print(f"Saved animation to {save_path}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Animate EKF x/y trajectory from one CSV file."
    )
    parser.add_argument("file", help="EKF CSV file")
    parser.add_argument(
        "--interval",
        type=int,
        default=40,
        help="Animation interval in milliseconds. Default: 40",
    )
    parser.add_argument(
        "--trail",
        type=int,
        default=None,
        help="Only show the last N points of the EKF trajectory",
    )
    parser.add_argument(
        "--save",
        default=None,
        help="Optional output file: animation.gif or animation.mp4",
    )

    args = parser.parse_args()

    animate_ekf_file(
        Path(args.file),
        interval_ms=args.interval,
        trail=args.trail,
        save_path=Path(args.save) if args.save else None,
    )


if __name__ == "__main__":
    main()