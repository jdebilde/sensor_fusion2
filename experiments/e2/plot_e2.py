#!/usr/bin/env python3
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

    plt.figure(figsize=(12, 6))
    plt.plot(df["time_s"], df["distance_cm"], marker=".", linewidth=1)

    for y in [50, 100, 200, 300, 400, 500]:
        plt.axhline(y=y, linestyle="--", linewidth=0.8)
        plt.text(df["time_s"].iloc[-1], y, f" {y} cm", va="center")

    tmp = path.name.split('_')
    title = f"uwb_{tmp[4]} to uwb_{tmp[7]} with delay={tmp[8]}"
    plt.title(title)
    plt.xlabel("Temps (s)")
    plt.ylabel("Distance (cm)")
    plt.grid(True)
    plt.tight_layout()
    svg = f"e2_{str(path).split('@')[1][:-4]}_{time()}.png"
    plt.savefig(svg)
    plt.show()

if __name__ == "__main__":
    main()