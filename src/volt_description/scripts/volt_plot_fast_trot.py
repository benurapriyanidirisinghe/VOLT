#!/usr/bin/env python3

"""Plot CSV recordings produced by ``volt_fast_trot_diagnostic.py``."""

import argparse
import csv
import math
from pathlib import Path
import sys


LEG_ORDER = (
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
)
JOINT_NAMES = (
    "front_left_shoulder",
    "front_left_leg",
    "front_left_foot",
    "front_right_shoulder",
    "front_right_leg",
    "front_right_foot",
    "rear_left_shoulder",
    "rear_left_leg",
    "rear_left_foot",
    "rear_right_shoulder",
    "rear_right_leg",
    "rear_right_foot",
)


def numeric(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return value if math.isfinite(value) else float("nan")


def load_recording(path):
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        fields = set(reader.fieldnames or [])
    required = {
        "elapsed_s",
        "cycle_phase",
        "requested_stride_m",
        "achieved_stride_m",
    }
    missing = sorted(required - fields)
    if missing:
        raise ValueError("CSV is missing required columns: %s" % missing)
    if not rows:
        raise ValueError("CSV contains no diagnostic samples")
    return rows, fields


def series(rows, key):
    return [numeric(row.get(key)) for row in rows]


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Plot a VOLT live fast-trot diagnostic CSV.",
    )
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output PDF path (default: CSV name with .pdf suffix).",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Also show interactive plot windows.",
    )
    return parser.parse_args(argv)


def build_figures(plt, rows, fields):
    time_values = series(rows, "elapsed_s")
    figures = []

    figure, axis = plt.subplots(figsize=(11, 6))
    for leg_name in LEG_ORDER:
        key = "%s_foot_x_m" % leg_name
        if key in fields:
            axis.plot(time_values, series(rows, key), label=leg_name)
    axis.set_title("Fast trot — commanded foot x")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Body-frame foot x (m)")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=2)
    figures.append(figure)

    figure, axis = plt.subplots(figsize=(11, 6))
    for leg_name in LEG_ORDER:
        key = "%s_foot_z_m" % leg_name
        if key in fields:
            axis.plot(time_values, series(rows, key), label=leg_name)
    axis.set_title("Fast trot — commanded foot z")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Body-frame foot z (m)")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=2)
    figures.append(figure)

    figure, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    for joint_name in JOINT_NAMES:
        key = "%s_rad" % joint_name
        if key not in fields:
            continue
        if joint_name.endswith("_shoulder"):
            axis = axes[0]
        elif joint_name.endswith("_leg"):
            axis = axes[1]
        else:
            axis = axes[2]
        axis.plot(time_values, series(rows, key), label=joint_name)
    axes[0].set_title("Canonical joint commands")
    for axis, label in zip(axes, ("Shoulder", "Leg", "Foot")):
        axis.set_ylabel("%s (rad)" % label)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=7, ncol=2)
    axes[-1].set_xlabel("Time (s)")
    figures.append(figure)

    figure, axis = plt.subplots(figsize=(11, 6))
    requested = series(rows, "requested_stride_m")
    achieved = series(rows, "achieved_stride_m")
    metric_valid = [
        str(row.get("stride_metric_valid", "true")).lower()
        in ("1", "true", "yes")
        for row in rows
    ]
    achieved = [
        value if valid else float("nan")
        for value, valid in zip(achieved, metric_valid)
    ]
    axis.plot(time_values, requested, label="requested")
    axis.plot(
        time_values,
        achieved,
        label="grounded achieved (near-straight X)",
    )
    if "signed_stride_m" in fields:
        signed = series(rows, "signed_stride_m")
        signed = [
            value if valid else float("nan")
            for value, valid in zip(signed, metric_valid)
        ]
        axis.plot(
            time_values,
            signed,
            linestyle="--",
            label="signed touchdown-to-liftoff X",
        )
    axis.set_title(
        "Requested versus grounded downstream stride "
        "(metric paused while turning)"
    )
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Stride (m)")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figures.append(figure)

    figure, axis = plt.subplots(figsize=(11, 6))
    tracking = series(rows, "joint_tracking_error_rad")
    if any(math.isfinite(value) for value in tracking):
        axis.plot(
            time_values,
            tracking,
            color="tab:red",
            label="tracking error",
        )
    else:
        axis.text(
            0.5,
            0.5,
            "Tracking feedback unavailable (open-loop recording)",
            ha="center",
            va="center",
            transform=axis.transAxes,
            color="tab:red",
        )
    axis.set_title("Joint tracking error and gait phase")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Tracking error (rad)", color="tab:red")
    axis.tick_params(axis="y", labelcolor="tab:red")
    axis.grid(True, alpha=0.3)
    phase_axis = axis.twinx()
    phase_axis.plot(
        time_values,
        series(rows, "cycle_phase"),
        color="tab:blue",
        alpha=0.65,
        label="cycle phase",
    )
    phase_axis.set_ylabel("Cycle phase", color="tab:blue")
    phase_axis.tick_params(axis="y", labelcolor="tab:blue")
    figures.append(figure)

    servo_minimum = []
    servo_maximum = []
    servo_labels = []
    for joint_name in JOINT_NAMES:
        key = "%s_servo_deg" % joint_name
        if key not in fields:
            continue
        values = [
            value for value in series(rows, key) if math.isfinite(value)
        ]
        if not values:
            continue
        servo_labels.append(joint_name.replace("_", "\n", 1))
        servo_minimum.append(min(values))
        servo_maximum.append(max(values))
    figure, axis = plt.subplots(figsize=(13, 7))
    indices = list(range(len(servo_labels)))
    excursions = [
        high - low for low, high in zip(servo_minimum, servo_maximum)
    ]
    axis.bar(indices, excursions, bottom=servo_minimum)
    for index, (low, high) in enumerate(
        zip(servo_minimum, servo_maximum)
    ):
        axis.text(
            index,
            high,
            "%.1f°" % (high - low),
            ha="center",
            va="bottom",
            fontsize=7,
        )
    axis.set_title("Calibrated servo command range during recording")
    axis.set_ylabel("Commanded servo angle (degrees)")
    axis.set_xticks(indices, servo_labels, rotation=45, ha="right")
    axis.grid(True, axis="y", alpha=0.3)
    figure.tight_layout()
    figures.append(figure)

    return figures


def main(argv=None):
    args = parse_arguments(argv)
    csv_path = args.csv_path.expanduser().resolve()
    if not csv_path.is_file():
        print("error: CSV file does not exist: %s" % csv_path, file=sys.stderr)
        return 2
    try:
        rows, fields = load_recording(csv_path)
    except (OSError, ValueError) as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 2

    try:
        import matplotlib
        if not args.show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_pdf import PdfPages
    except ImportError:
        print(
            "error: matplotlib is unavailable; install python3-matplotlib "
            "to create fast-trot plots.",
            file=sys.stderr,
        )
        return 2

    output = (
        args.output.expanduser().resolve()
        if args.output
        else csv_path.with_suffix(".pdf")
    )
    if output.suffix.lower() != ".pdf":
        output = output.with_suffix(".pdf")
    output.parent.mkdir(parents=True, exist_ok=True)

    figures = build_figures(plt, rows, fields)
    with PdfPages(output) as pdf:
        for figure in figures:
            pdf.savefig(figure, bbox_inches="tight")
    print("wrote %d diagnostic plots to %s" % (len(figures), output))

    if args.show:
        plt.show()
    else:
        for figure in figures:
            plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
