#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


TAG_POSITIONS = [
    (-1.9, 3.0),
    (-1.0, 3.0),
    (0.0, 3.0),
    (1.0, 3.0),
    (2.0, 3.0),
]

MEASURES_PER_POSITION = 50


def true_distance_cm(
    anchor_x: float,
    anchor_y: float,
    tag_x: float,
    tag_y: float,
) -> float:
    """Calcule la distance réelle entre l'ancre et le tag en centimètres."""
    distance_m = math.hypot(tag_x - anchor_x, tag_y - anchor_y)
    return distance_m * 100.0


def correct_distance(d):
    points = [
        (37.06, 50.0),
        (93.61, 100.0),
        (206.44, 200.0),
        (308.42, 300.0),
        (419.00, 400.0),
        (515.25, 500.0),
    ]

    return interpolate(d, points)


def interpolate(d, points):
    # Below calibration range:
    # extrapolate using the first segment
    if d < points[0][0]:
        x1, y1 = points[0]
        x2, y2 = points[1]
        return linear_interpolate(d, x1, y1, x2, y2)

    # Inside calibration range
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]

        if x1 <= d <= x2:
            return linear_interpolate(d, x1, y1, x2, y2)

    # Above calibration range:
    # extrapolate using the last segment
    x1, y1 = points[-2]
    x2, y2 = points[-1]
    return linear_interpolate(d, x1, y1, x2, y2)


def linear_interpolate(d, x1, y1, x2, y2):
    return y1 + (d - x1) * (y2 - y1) / (x2 - x1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Affiche les mesures UWB par position et les compare "
            "aux distances réelles."
        )
    )

    parser.add_argument(
        "file",
        help="Chemin vers le fichier CSV UWB",
    )

    parser.add_argument(
        "--anchor-x",
        type=float,
        required=True,
        help="Position x de l'ancre en mètres",
    )

    parser.add_argument(
        "--anchor-y",
        type=float,
        required=True,
        help="Position y de l'ancre en mètres",
    )

    parser.add_argument(
        "--save",
        help="Chemin optionnel pour sauvegarder le graphique",
    )

    args = parser.parse_args()

    csv_path = Path(args.file)

    if not csv_path.exists():
        raise FileNotFoundError(f"Fichier introuvable : {csv_path}")

    df = pd.read_csv(
        csv_path,
        header=None,
        names=[
            "seq",
            "timestamp_ms",
            "anchor_id",
            "distance_cm",
            "x",
            "y",
        ],
    )

    expected_count = len(TAG_POSITIONS) * MEASURES_PER_POSITION

    if len(df) < expected_count:
        raise ValueError(
            f"Le fichier contient {len(df)} mesures, "
            f"mais au moins {expected_count} sont attendues."
        )

    if len(df) > expected_count:
        print(
            f"Attention : le fichier contient {len(df)} mesures. "
            f"Seules les {expected_count} premières seront utilisées."
        )
        df = df.iloc[:expected_count].copy()

    plt.figure(figsize=(14, 7))

    true_distances = []
    means = []
    stdevs = []

    for position_index, (tag_x, tag_y) in enumerate(TAG_POSITIONS):
        start = position_index * MEASURES_PER_POSITION
        end = start + MEASURES_PER_POSITION

        block = df.iloc[start:end]

        # These tests were done with no correction, so we can apply it here
        for i in range(start, end):
            df.loc[i, "distance_cm"] = correct_distance(block["distance_cm"][i])

        # Axe x global : 0 à 249
        sample_indices = list(range(start, end))

        distance_real_cm = true_distance_cm(
            args.anchor_x,
            args.anchor_y,
            tag_x,
            tag_y,
        )

        true_distances.append(distance_real_cm)

        measurements = df["distance_cm"].iloc[start:end]
        mean = measurements.mean()
        stdev = measurements.std()
        means.append(round(mean, 2))
        stdevs.append(round(stdev, 2))

        # Les 50 mesures du bloc
        plt.plot(
            sample_indices,
            block["distance_cm"],
            marker=".",
            linestyle="none",
            label=f"Tag measurements ({tag_x:g}, {tag_y:g})",
        )

        # Distance réelle sur tout le bloc
        plt.hlines(
            y=distance_real_cm,
            xmin=start,
            xmax=end - 1,
            linewidth=2,
            label=f"Actual distance ({tag_x:g}, {tag_y:g})",
        )

        # Séparation visuelle entre les positions
        if position_index > 0:
            plt.axvline(
                x=start - 0.5,
                linestyle="--",
                linewidth=0.8,
            )
    
    for i in range(len(TAG_POSITIONS)):
        true_distances[i] = round(float(true_distances[i]), 2)
        means[i] = round(float(means[i]), 2)
        stdevs[i] = round(float(stdevs[i]), 2)
    print("TAG_POSITIONS", TAG_POSITIONS)
    print("true_distances", true_distances)
    print("means", means)
    print("stdevs", stdevs)

    # Un tick au centre de chaque bloc
    tick_positions = [
        index * MEASURES_PER_POSITION
        + (MEASURES_PER_POSITION - 1) / 2
        for index in range(len(TAG_POSITIONS))
    ]

    tick_labels = [
        f"({x:g}, {y:g})"
        for x, y in TAG_POSITIONS
    ]

    plt.xticks(tick_positions, tick_labels)

    plt.xlabel("Actual position of the tag (m)")
    plt.ylabel("Measured distance (cm)")
    plt.title(
        f"UWB Measurements — Anchor ({args.anchor_x:g}, {args.anchor_y:g})"
    )

    plt.grid(True, axis="y")
    plt.legend(ncol=1)
    plt.tight_layout()

    if args.save:
        output_path = Path(args.save)
        plt.savefig(output_path, dpi=200)
        print(f"Chart saved in : {output_path}")

    plt.show()


if __name__ == "__main__":
    main()