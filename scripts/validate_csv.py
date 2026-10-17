#!/usr/bin/env python3
"""Validate a vehiclesim telemetry CSV against the scenario that produced it.

Uses only the Python standard library so it can run anywhere CI can.

Checks:
  * exact header and 7 finite numeric fields per row
  * first row is the scenario's initial state at t = 0
  * timestamps strictly increase, advance by the configured time step
    (only the final step may be shorter) and end exactly at the duration
  * speed stays within [0, max_speed]
  * steering and acceleration stay within the vehicle limits

Exit status: 0 if valid, 1 if any check fails, 2 on usage errors.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

EXPECTED_HEADER = ["time", "x", "y", "heading", "speed", "steering", "acceleration"]

# Absolute tolerance on time comparisons. The CSV stores 12 significant
# digits, so a parsed timestamp can differ from the exact grid value by
# ~1e-11 s for durations up to ~100 s.
TIME_TOLERANCE = 1e-9
# Tolerance on limit checks, covering the 12-significant-digit formatting.
LIMIT_TOLERANCE = 1e-9


class ValidationError(Exception):
    pass


def load_rows(csv_path: Path) -> list[dict[str, float]]:
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration:
            raise ValidationError("file is empty") from None
        if header != EXPECTED_HEADER:
            raise ValidationError(f"header {header} != expected {EXPECTED_HEADER}")

        rows = []
        for line_number, fields in enumerate(reader, start=2):
            if len(fields) != len(EXPECTED_HEADER):
                raise ValidationError(f"line {line_number}: expected 7 fields, got {len(fields)}")
            try:
                values = [float(field) for field in fields]
            except ValueError as error:
                raise ValidationError(f"line {line_number}: {error}") from None
            if not all(math.isfinite(value) for value in values):
                raise ValidationError(f"line {line_number}: non-finite value")
            rows.append(dict(zip(EXPECTED_HEADER, values)))
    if len(rows) < 2:
        raise ValidationError(f"expected at least 2 data rows, got {len(rows)}")
    return rows


def check_initial_state(rows: list[dict[str, float]], scenario: dict) -> None:
    first = rows[0]
    if first["time"] != 0.0:
        raise ValidationError(f"first timestamp is {first['time']}, expected 0")
    for key in ("x", "y", "heading", "speed"):
        expected = scenario["initial_state"][key]
        if not math.isclose(first[key], expected, rel_tol=1e-11, abs_tol=1e-12):
            raise ValidationError(f"initial {key} is {first[key]}, expected {expected}")


def expected_step_count(duration: float, time_step: float) -> int:
    """Mirror of stepCount() in src/simulator.cpp: duration / dt is treated
    as an integer when within a *relative* 1e-9, otherwise rounded up."""
    ratio = duration / time_step
    nearest = round(ratio)
    if abs(ratio - nearest) <= 1e-9 * max(1.0, ratio):
        return int(nearest)
    return math.ceil(ratio)


def check_timestamps(rows: list[dict[str, float]], scenario: dict) -> None:
    duration = scenario["simulation"]["duration"]
    time_step = scenario["simulation"]["time_step"]
    times = [row["time"] for row in rows]

    for index in range(1, len(times)):
        step = times[index] - times[index - 1]
        if step <= 0.0:
            raise ValidationError(f"row {index}: time does not increase ({times[index - 1]} -> {times[index]})")
        is_last = index == len(times) - 1
        if is_last:
            # The simulator absorbs a residual of up to 1e-9 * duration into
            # the final step instead of emitting a vanishingly short extra step.
            if step > time_step + TIME_TOLERANCE + 1e-9 * duration:
                raise ValidationError(f"final step {step} exceeds time_step {time_step}")
        elif abs(step - time_step) > TIME_TOLERANCE:
            raise ValidationError(f"row {index}: step {step} != time_step {time_step}")

    if abs(times[-1] - duration) > TIME_TOLERANCE:
        raise ValidationError(f"final timestamp {times[-1]} != duration {duration}")

    expected_rows = expected_step_count(duration, time_step) + 1
    if len(rows) != expected_rows:
        raise ValidationError(f"{len(rows)} data rows, expected {expected_rows}")


def check_limits(rows: list[dict[str, float]], scenario: dict) -> None:
    vehicle = scenario["vehicle"]
    for index, row in enumerate(rows):
        if not -LIMIT_TOLERANCE <= row["speed"] <= vehicle["max_speed"] + LIMIT_TOLERANCE:
            raise ValidationError(f"row {index}: speed {row['speed']} outside [0, {vehicle['max_speed']}]")
        if abs(row["steering"]) > vehicle["max_steering_angle"] + LIMIT_TOLERANCE:
            raise ValidationError(f"row {index}: |steering| {abs(row['steering'])} > {vehicle['max_steering_angle']}")
        if not -vehicle["max_braking"] - LIMIT_TOLERANCE <= row["acceleration"] <= vehicle["max_acceleration"] + LIMIT_TOLERANCE:
            raise ValidationError(f"row {index}: acceleration {row['acceleration']} outside limits")


def validate(csv_path: Path, scenario_path: Path) -> int:
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    rows = load_rows(csv_path)
    check_initial_state(rows, scenario)
    check_timestamps(rows, scenario)
    check_limits(rows, scenario)
    return len(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", type=Path, help="telemetry CSV written by vehiclesim")
    parser.add_argument("--scenario", type=Path, required=True, help="scenario JSON that produced the CSV")
    args = parser.parse_args()

    for path in (args.csv, args.scenario):
        if not path.is_file():
            print(f"error: {path}: no such file", file=sys.stderr)
            return 2
    try:
        row_count = validate(args.csv, args.scenario)
    except ValidationError as error:
        print(f"FAIL {args.csv}: {error}", file=sys.stderr)
        return 1
    print(f"OK   {args.csv}: {row_count} rows valid against {args.scenario}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
