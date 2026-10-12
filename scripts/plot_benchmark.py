#!/usr/bin/env python3
"""Plot UMBmark benchmark rows and animate before/after square trajectories."""

import argparse
import csv
from collections import OrderedDict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


ROOT = Path(__file__).resolve().parents[1]


def load_rows(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def trial_groups(rows):
    groups = OrderedDict()
    for row in rows:
        if row["row_type"] != "trial":
            continue
        key = (row["ed_true"], row["eb_true"])
        groups.setdefault(key, []).append(row)
    return groups


def mean(values):
    return sum(values) / len(values)


def stdev(values):
    if len(values) < 2:
        return 0.0
    mu = mean(values)
    return (sum((value - mu) ** 2 for value in values) / (len(values) - 1)) ** 0.5


def plot_emax(rows, output):
    groups = trial_groups(rows)
    labels = [f"Ed {float(ed):.3f}\nEb {float(eb):.3f}" for ed, eb in groups]
    before = []
    before_std = []
    umb = []
    umb_std = []
    least = []
    least_std = []
    for samples in groups.values():
        b = [float(sample["e_max_before_m"]) for sample in samples]
        u = [float(sample["e_max_after_umb_m"]) for sample in samples]
        ell = [float(sample["e_max_after_ls_m"]) for sample in samples]
        before.append(mean(b))
        before_std.append(stdev(b))
        umb.append(mean(u))
        umb_std.append(stdev(u))
        least.append(mean(ell))
        least_std.append(stdev(ell))

    xpos = list(range(len(labels)))
    width = 0.26
    fig, ax = plt.subplots(figsize=(8.4, 4.6))
    ax.bar(
        [x - width for x in xpos],
        before,
        width,
        yerr=before_std,
        capsize=3,
        label="Before",
        color="#4c78a8",
    )
    ax.bar(xpos, umb, width, yerr=umb_std, capsize=3, label="After UMBmark", color="#f58518")
    ax.bar(
        [x + width for x in xpos],
        least,
        width,
        yerr=least_std,
        capsize=3,
        label="After least squares",
        color="#54a24b",
    )
    ax.set_xticks(xpos)
    ax.set_xticklabels(labels)
    ax.set_ylabel("E_max,syst (m)")
    ax.set_xlabel("Injected systematic error")
    ax.set_title("Return-position error on held-out bidirectional squares")
    ax.legend(frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.yaxis.grid(True, linestyle=":", linewidth=0.6)
    ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(output, dpi=140)
    plt.close(fig)


def square_polyline(side, direction):
    if direction == "ccw":
        corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
    else:
        corners = [(0.0, 0.0), (side, 0.0), (side, -side), (0.0, -side), (0.0, 0.0)]
    return [point[0] for point in corners], [point[1] for point in corners]


def load_traces(path):
    traces = {}
    meta = None
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            key = (row["direction"], row["trace"])
            traces.setdefault(key, []).append(
                (int(row["index"]), float(row["x_m"]), float(row["y_m"]))
            )
            meta = row
    for points in traces.values():
        points.sort(key=lambda item: item[0])
    return traces, meta


def animate(traces, meta, output):
    side = float(meta["side_m"])
    ed = meta["ed_true"]
    eb = meta["eb_true"]
    directions = ("cw", "ccw")
    needed = []
    for direction in directions:
        for trace in ("actual", "odom_before", "odom_umbmark", "odom_least_squares"):
            needed.append(traces[(direction, trace)])
    length = len(needed[0])
    stride = max(1, length // 70)
    frames = list(range(1, length, stride))
    if frames[-1] != length:
        frames.append(length)
    hold = frames[-1]
    frames.extend([hold] * 8)

    fig, axes = plt.subplots(1, 2, figsize=(8.6, 4.8), sharex=True, sharey=True)
    colors = {"cw": "#1f77b4", "ccw": "#d62728"}

    for ax, title in zip(axes, ("Before calibration", "After calibration")):
        for direction in directions:
            xs, ys = square_polyline(side, direction)
            ax.plot(xs, ys, color="#d0d0d0", linewidth=1.0, zorder=0)
        ax.set_title(title)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x (m)")
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.grid(True, linestyle=":", linewidth=0.5)
    axes[0].set_ylabel("y (m)")

    def add_line(ax, color, style, label):
        (line,) = ax.plot([], [], color=color, linestyle=style, linewidth=1.6, label=label)
        return line

    left_actual = {}
    left_odom = {}
    right_actual = {}
    right_umb = {}
    right_ls = {}
    for direction in directions:
        left_actual[direction] = add_line(axes[0], colors[direction], "-", f"truth {direction.upper()}")
        left_odom[direction] = add_line(axes[0], colors[direction], "--", f"odom {direction.upper()}")
        right_actual[direction] = add_line(axes[1], colors[direction], "-", f"truth {direction.upper()}")
        right_umb[direction] = add_line(axes[1], "#f58518", "--", f"UMBmark {direction.upper()}")
        right_ls[direction] = add_line(axes[1], "#54a24b", ":", f"least squares {direction.upper()}")

    axes[0].legend(frameon=False, fontsize=8, loc="upper left")
    axes[1].legend(frameon=False, fontsize=8, loc="upper left")
    fig.suptitle(f"Square side {side:.3f} m, Ed = {float(ed):.3f}, Eb = {float(eb):.3f}")
    fig.tight_layout()

    def slice_xy(direction, trace, count):
        points = traces[(direction, trace)][:count]
        return [point[1] for point in points], [point[2] for point in points]

    def update(count):
        drawn = []
        for direction in directions:
            xs, ys = slice_xy(direction, "actual", count)
            left_actual[direction].set_data(xs, ys)
            right_actual[direction].set_data(xs, ys)
            xs, ys = slice_xy(direction, "odom_before", count)
            left_odom[direction].set_data(xs, ys)
            xs, ys = slice_xy(direction, "odom_umbmark", count)
            right_umb[direction].set_data(xs, ys)
            xs, ys = slice_xy(direction, "odom_least_squares", count)
            right_ls[direction].set_data(xs, ys)
            drawn.extend(
                [
                    left_actual[direction],
                    left_odom[direction],
                    right_actual[direction],
                    right_umb[direction],
                    right_ls[direction],
                ]
            )
        return drawn

    # Pad limits so both squares and the drifted paths stay in frame.
    xs = []
    ys = []
    for points in traces.values():
        xs.extend(point[1] for point in points)
        ys.extend(point[2] for point in points)
    pad = 0.4
    for ax in axes:
        ax.set_xlim(min(xs) - pad, max(xs) + pad)
        ax.set_ylim(min(ys) - pad, max(ys) + pad)

    animation = FuncAnimation(fig, update, frames=frames, interval=70, blit=False)
    animation.save(output, writer=PillowWriter(fps=12))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=ROOT / "docs" / "benchmark.csv")
    parser.add_argument("--traj", type=Path, default=ROOT / "docs" / "trajectories.csv")
    parser.add_argument("--plot", type=Path, default=ROOT / "docs" / "emax_before_after.png")
    parser.add_argument("--gif", type=Path, default=ROOT / "docs" / "trajectories.gif")
    args = parser.parse_args()
    rows = load_rows(args.csv)
    plot_emax(rows, args.plot)
    traces, meta = load_traces(args.traj)
    animate(traces, meta, args.gif)
    print(f"wrote {args.plot}")
    print(f"wrote {args.gif}")


if __name__ == "__main__":
    main()
