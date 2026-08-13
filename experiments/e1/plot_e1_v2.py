#!/usr/bin/env python3
# python3 plot_e1_v2.py uwb_measure_sensor_fusion@uwb_3_to_uwb_2_16415_n2.csv
import argparse
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from time import time

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="CSV UWB à plotter")
    args = parser.parse_args()

    path = Path(args.file)

    df = pd.read_csv(path, header=None, names=[
        "seq", "timestamp_ms", "anchor", "distance_cm", "x", "y"
    ])

    # Temps relatif en secondes
    df["time_s"] = (df["timestamp_ms"] - df["timestamp_ms"].iloc[0]) / 1000.0
    for i in range(len(df["time_s"])):
        # print(df["time_s"][i], i * 75 / 1000)
        df.loc[i, "time_s"] = i * 75 / 1000

    plt.figure(figsize=(12, 6))
    plt.plot(df["time_s"], df["distance_cm"], marker=".", linewidth=0)

    distance = [50, 100, 200, 300, 400, 500]
    for y in distance:
        plt.axhline(y=y, linestyle="--", linewidth=1, color="black")
        # plt.text(df["time_s"].iloc[-1], y, f" {y} cm", va="center")

    means = []
    stdevs = []

    N = 50

    for y in range(6):
        start = y * N
        end = start + N

        measurements = df["distance_cm"].iloc[start:end]

        mean = measurements.mean()
        stdev = measurements.std()

        means.append(round(mean, 2))
        stdevs.append(round(stdev, 2))

    for idx, y in enumerate(means):
        bias = means[idx] - distance[idx]

        print(
            f"{distance[idx]:3d} cm : "
            f"mean={means[idx]:7.2f} cm, "
            f"bias={bias:+7.2f} cm, "
            f"stdev={stdevs[idx]:5.2f} cm"
        )
        # print(f"[{y}, {distance[idx]}],")
        # print(f"{y} & ", end='')

    # for idx, y in enumerate(means):
    #     plt.axhline(y=y, linestyle="--", linewidth=1, color="blue")
    #     plt.text(df["time_s"].iloc[idx * N + 25], y + 30, f" {y} cm", va="center")
    for idx, mean in enumerate(means):
        start = idx * N
        end = start + N - 1

        x_start = df["time_s"].iloc[start]
        x_end = df["time_s"].iloc[end]

        # Moyenne
        plt.hlines(
            mean,
            x_start,
            x_end,
            linewidth=1,
            color="blue"
        )

        # Mean ± standard deviation
        plt.fill_between(
            [x_start, x_end],
            mean - stdevs[idx],
            mean + stdevs[idx],
            alpha=0.2
        )

        plt.text(
            (x_start + x_end) / 2,
            mean + 10,
            f"{mean:.2f} ± {stdevs[idx]:.2f} cm",
            ha="center"
        )

    tmp = path.name.split('_')
    title = f"uwb_{tmp[4]} to uwb_{tmp[7]} with delay={tmp[8]}"
    plt.title(title)
    plt.xlabel("Temps (s)")
    plt.ylabel("Distance (cm)")
    plt.grid(True)
    plt.tight_layout()
    png = f"e1_{str(path).split('@')[1][:-4]}_{time()}.png"
    plt.savefig(png)
    plt.show()

if __name__ == "__main__":
    main()