#!/usr/bin/env python3
"""Small calculations shared by the saved ArduPilot reports."""

import csv
import math
import statistics


def read_rows(path):
    with path.open(newline="") as source:
        return list(csv.DictReader(source))


def lockstep_summary(exchange):
    if len(exchange) < 2:
        raise RuntimeError("exchange log does not contain enough controller frames")
    frames = [int(row["servo_frame"]) for row in exchange]
    steps = [int(row["step"]) for row in exchange]
    physics_times = [float(row["physics_time_s"]) for row in exchange]
    rates = {int(row["frame_rate_hz"]) for row in exchange}
    frame_gaps = [right - left for left, right in zip(frames, frames[1:])]
    step_gaps = [right - left for left, right in zip(steps, steps[1:])]
    time_steps = [right - left for left, right in zip(physics_times, physics_times[1:])]
    rate_hz = next(iter(rates)) if len(rates) == 1 else 0
    fixed_dt_s = statistics.median(time_steps)
    missed = max(int(row["missed_frames"]) for row in exchange)
    duplicates = max(int(row["duplicate_frames"]) for row in exchange)
    verified = (
        all(gap == 1 for gap in frame_gaps)
        and all(gap == 1 for gap in step_gaps)
        and missed == 0
        and duplicates == 0
        and rate_hz > 0
        and abs(fixed_dt_s - 1.0 / rate_hz) < 1.0e-6
    )
    return {
        "verified": verified,
        "frames": len(exchange),
        "rate_hz": rate_hz,
        "fixed_dt_s": fixed_dt_s,
        "missed": missed,
        "duplicates": duplicates,
    }


def dual_lockstep_summary(left_exchange, right_exchange):
    if not left_exchange or not right_exchange:
        raise RuntimeError("dual exchange logs do not contain controller frames")
    count = min(len(left_exchange), len(right_exchange))
    step_skews = [
        abs(int(left_exchange[index]["step"]) - int(right_exchange[index]["step"]))
        for index in range(count)
    ]
    time_skews = [
        abs(
            float(left_exchange[index]["physics_time_s"])
            - float(right_exchange[index]["physics_time_s"])
        )
        for index in range(count)
    ]
    return {
        "same_frame_count": len(left_exchange) == len(right_exchange),
        "max_step_skew": max(step_skews),
        "max_time_skew_s": max(time_skews),
    }


def run_coverage_summary(exchange, truth, duration_s, fixed_dt_s, truth_rate_hz=20):
    if duration_s <= 0.0 or fixed_dt_s <= 0.0 or truth_rate_hz <= 0:
        raise RuntimeError("run coverage settings must be positive")
    expected_steps = math.floor(duration_s / fixed_dt_s + 0.5)
    truth_interval = max(1, math.floor(1.0 / (truth_rate_hz * fixed_dt_s) + 0.5))
    if expected_steps < 1:
        raise RuntimeError("run duration must contain at least one fixed step")
    expected_final_time_s = expected_steps * fixed_dt_s
    expected_truth_frames = expected_steps // truth_interval + 1
    time_tolerance_s = max(1.0e-9, 0.01 * fixed_dt_s)
    exchange_times = [float(row["time_s"]) for row in exchange]
    truth_times = [float(row["time_s"]) for row in truth]
    exchange_complete = (
        bool(exchange_times)
        and len(exchange) == expected_steps
        and abs(exchange_times[0] - fixed_dt_s) <= time_tolerance_s
        and abs(exchange_times[-1] - expected_final_time_s) <= time_tolerance_s
    )
    truth_complete = (
        bool(truth_times)
        and len(truth) == expected_truth_frames
        and abs(truth_times[0]) <= time_tolerance_s
        and abs(truth_times[-1] - expected_final_time_s) <= time_tolerance_s
    )
    return {
        "exchange_complete": exchange_complete,
        "truth_complete": truth_complete,
        "expected_exchange_frames": expected_steps,
        "expected_truth_frames": expected_truth_frames,
        "expected_final_time_s": expected_final_time_s,
    }
