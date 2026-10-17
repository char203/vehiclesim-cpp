#!/usr/bin/env python3
"""Plot vehiclesim telemetry CSVs.

For every CSV this writes four PNGs into the output directory:

  <name>_trajectory.png  x-y path of the rear axle (equal axis scaling)
  <name>_speed.png       speed versus time
  <name>_heading.png     heading versus time (degrees, unwrapped)
  <name>_lateral.png     y position versus time (makes small lateral
                         offsets, e.g. a lane change, readable)
  <name>_overview.png    all four panels side by side

With --overlay, all CSVs are drawn on shared axes instead (e.g. to compare
Euler and RK4 runs of one scenario) and written as overlay_*.png.

Requires matplotlib (see scripts/requirements.txt). Rendering uses the
non-interactive Agg backend, so no display is needed.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (backend must be chosen first)

EXPECTED_HEADER = ["time", "x", "y", "heading", "speed", "steering", "acceleration"]

# Validated categorical order (blue, orange, aqua); hues are assigned by
# position and never cycled. Line styles add a non-color cue for overlays.
SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a"]
SERIES_STYLES = ["-", "--", ":"]
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"


@dataclass(frozen=True)
class Telemetry:
    label: str
    time: list[float]
    x: list[float]
    y: list[float]
    heading_deg: list[float]
    speed: list[float]


def load_telemetry(path: Path, label: str | None = None) -> Telemetry:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if header != EXPECTED_HEADER:
            raise ValueError(f"{path}: unexpected header {header}")
        columns: dict[str, list[float]] = {name: [] for name in EXPECTED_HEADER}
        for line_number, fields in enumerate(reader, start=2):
            if len(fields) != len(EXPECTED_HEADER):
                raise ValueError(f"{path}:{line_number}: expected 7 fields, got {len(fields)}")
            for name, field in zip(EXPECTED_HEADER, fields):
                columns[name].append(float(field))
    if not columns["time"]:
        raise ValueError(f"{path}: no data rows")
    return Telemetry(
        label=label or path.stem,
        time=columns["time"],
        x=columns["x"],
        y=columns["y"],
        heading_deg=[math.degrees(value) for value in columns["heading"]],
        speed=columns["speed"],
    )


def style_axes(ax: plt.Axes, title: str, xlabel: str, ylabel: str) -> None:
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=TEXT_PRIMARY, fontsize=11, loc="left")
    ax.set_xlabel(xlabel, color=TEXT_SECONDARY)
    ax.set_ylabel(ylabel, color=TEXT_SECONDARY)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.8, linestyle="-")
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def plot_trajectory(ax: plt.Axes, runs: list[Telemetry], mark_endpoints: bool) -> None:
    for index, run in enumerate(runs):
        ax.plot(run.x, run.y, color=SERIES_COLORS[index], linestyle=SERIES_STYLES[index], linewidth=2,
                solid_capstyle="round", label=run.label)
    if mark_endpoints:
        run = runs[0]
        for (px, py), marker, text in (((run.x[0], run.y[0]), "o", "start"), ((run.x[-1], run.y[-1]), "s", "end")):
            ax.plot(px, py, marker=marker, markersize=8, color=SERIES_COLORS[0],
                    markeredgecolor=SURFACE, markeredgewidth=2, linestyle="none", zorder=3)
            ax.annotate(text, (px, py), xytext=(6, 6), textcoords="offset points", color=TEXT_SECONDARY, fontsize=9)
    # Equal scaling: one metre is the same length on both axes, so curvature
    # and lateral offsets are not visually distorted.
    ax.set_aspect("equal", adjustable="datalim")
    style_axes(ax, "Trajectory (rear axle)", "x [m]", "y [m]")


def plot_series(ax: plt.Axes, runs: list[Telemetry], field: str, title: str, ylabel: str,
                zero_based: bool = False) -> None:
    for index, run in enumerate(runs):
        ax.plot(run.time, getattr(run, field), color=SERIES_COLORS[index], linestyle=SERIES_STYLES[index],
                linewidth=2, label=run.label)
    if zero_based:
        # Magnitudes start at zero so a constant value reads as constant
        # instead of being stretched across an auto-zoomed axis.
        top = max(max(getattr(run, field)) for run in runs)
        ax.set_ylim(0.0, max(top * 1.1, 1.0))
    style_axes(ax, title, "time [s]", ylabel)


def add_legend(ax: plt.Axes, runs: list[Telemetry]) -> None:
    if len(runs) >= 2:
        ax.legend(frameon=False, labelcolor=TEXT_PRIMARY, fontsize=9)


def save(fig: plt.Figure, path: Path) -> Path:
    fig.patch.set_facecolor(SURFACE)
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return path


def render(runs: list[Telemetry], name: str, output_dir: Path) -> list[Path]:
    endpoints = len(runs) == 1
    written = []

    fig, ax = plt.subplots(figsize=(6.5, 5))
    plot_trajectory(ax, runs, endpoints)
    add_legend(ax, runs)
    written.append(save(fig, output_dir / f"{name}_trajectory.png"))

    fig, ax = plt.subplots(figsize=(6.5, 3.5))
    plot_series(ax, runs, "speed", "Speed", "speed [m/s]", zero_based=True)
    add_legend(ax, runs)
    written.append(save(fig, output_dir / f"{name}_speed.png"))

    fig, ax = plt.subplots(figsize=(6.5, 3.5))
    plot_series(ax, runs, "heading_deg", "Heading", "heading [deg]")
    add_legend(ax, runs)
    written.append(save(fig, output_dir / f"{name}_heading.png"))

    fig, ax = plt.subplots(figsize=(6.5, 3.5))
    plot_series(ax, runs, "y", "Lateral position", "y [m]")
    add_legend(ax, runs)
    written.append(save(fig, output_dir / f"{name}_lateral.png"))

    fig, axes = plt.subplots(1, 4, figsize=(19, 4.2))
    plot_trajectory(axes[0], runs, endpoints)
    plot_series(axes[1], runs, "speed", "Speed", "speed [m/s]", zero_based=True)
    plot_series(axes[2], runs, "heading_deg", "Heading", "heading [deg]")
    plot_series(axes[3], runs, "y", "Lateral position", "y [m]")
    add_legend(axes[0], runs)
    fig.suptitle(name, color=TEXT_PRIMARY, fontsize=13, x=0.01, ha="left")
    written.append(save(fig, output_dir / f"{name}_overview.png"))
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+", type=Path, help="telemetry CSV file(s) written by vehiclesim")
    parser.add_argument("-o", "--output-dir", type=Path, default=Path("output/plots"),
                        help="directory for PNG files (default: output/plots)")
    parser.add_argument("--overlay", action="store_true", help="draw all CSVs on shared axes")
    parser.add_argument("--label", dest="labels", action="append",
                        help="legend label for --overlay, repeat once per CSV in order (default: file names)")
    args = parser.parse_args()

    if args.overlay and len(args.csv) > len(SERIES_COLORS):
        parser.error(f"--overlay supports at most {len(SERIES_COLORS)} files")
    if args.labels and len(args.labels) != len(args.csv):
        parser.error("--label must be given once per CSV")

    try:
        labels = args.labels or [None] * len(args.csv)
        runs = [load_telemetry(path, label) for path, label in zip(args.csv, labels)]
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    written = []
    if args.overlay:
        written += render(runs, "overlay", args.output_dir)
    else:
        for run in runs:
            written += render([run], run.label, args.output_dir)
    for path in written:
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
