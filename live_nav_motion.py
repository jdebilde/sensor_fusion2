#!/usr/bin/env python3
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from collections import deque


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--window", type=int, default=300)
    parser.add_argument("--dt", type=float, default=0.05, help="fallback dt en secondes")
    args = parser.parse_args()

    t = deque(maxlen=args.window)
    ax = deque(maxlen=args.window)
    ay = deque(maxlen=args.window)
    omega = deque(maxlen=args.window)

    prev_vx = prev_vy = None
    prev_ts = None
    t0 = None

    plt.ion()
    fig, axs = plt.subplots(3, 1, sharex=True)

    l_ax, = axs[0].plot([], [])
    l_ay, = axs[1].plot([], [])
    l_om, = axs[2].plot([], [])

    axs[0].set_ylabel("ax [m/s²]")
    axs[1].set_ylabel("ay [m/s²]")
    axs[2].set_ylabel("omega [deg/s]")
    axs[2].set_xlabel("temps [s]")

    for a in axs:
        a.grid(True)

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue

        try:
            seq, ts, x, y, theta, vx, vy, om = map(float, line.split(","))
        except ValueError:
            continue

        if t0 is None:
            t0 = ts

        now = (ts - t0) / 1000.0

        if prev_ts is None:
            prev_ts = ts
            prev_vx = vx
            prev_vy = vy
            continue

        dt = (ts - prev_ts) / 1000.0
        if dt <= 0 or dt > 1.0:
            dt = args.dt

        acc_x = (vx - prev_vx) / dt
        acc_y = (vy - prev_vy) / dt
        om_deg = np.rad2deg(om)

        t.append(now)
        ax.append(acc_x)
        ay.append(acc_y)
        omega.append(om_deg)

        prev_ts = ts
        prev_vx = vx
        prev_vy = vy

        l_ax.set_data(t, ax)
        l_ay.set_data(t, ay)
        l_om.set_data(t, omega)

        for a in axs:
            a.relim()
            a.autoscale_view()

        plt.pause(0.00001)


if __name__ == "__main__":
    main()