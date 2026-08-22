#!/usr/bin/env python3
"""Validate and plot the bounded ArduRover ACRO boat run."""

import csv
import html
import math
from pathlib import Path
import statistics
import sys

import plotly.graph_objects as go
import plotly.io as pio
from plotly.subplots import make_subplots


BLUE = "#2563eb"
RED = "#dc2626"
SLATE = "#64748b"
ORANGE = "#ea580c"


def read_rows(path):
    with path.open(newline="") as source:
        return list(csv.DictReader(source))


def values(rows, field):
    return [float(row[field]) for row in rows]


def phase_rows(rows, phase):
    return [row for row in rows if row["phase"] == phase]


def median(rows, field):
    selected = values(rows, field)
    return statistics.median(selected) if selected else math.nan


def lockstep_summary(exchange):
    if len(exchange) < 2:
        raise RuntimeError("exchange log does not contain enough controller frames")
    frames = [int(row["servo_frame"]) for row in exchange]
    steps = [int(row["step"]) for row in exchange]
    physics_times = values(exchange, "physics_time_s")
    rates = {int(row["frame_rate_hz"]) for row in exchange}
    time_steps = [right - left for left, right in zip(physics_times, physics_times[1:])]
    rate_hz = next(iter(rates)) if len(rates) == 1 else 0
    fixed_dt_s = statistics.median(time_steps)
    missed = max(int(row["missed_frames"]) for row in exchange)
    duplicates = max(int(row["duplicate_frames"]) for row in exchange)
    verified = (
        all(right - left == 1 for left, right in zip(frames, frames[1:]))
        and all(right - left == 1 for left, right in zip(steps, steps[1:]))
        and missed == 0
        and duplicates == 0
        and rate_hz > 0
        and abs(fixed_dt_s - 1.0 / rate_hz) < 1.0e-6
    )
    return {
        "verified": verified,
        "frames": len(exchange),
        "rate_hz": rate_hz,
        "missed": missed,
        "duplicates": duplicates,
    }


def make_plot(truth, exchange, telemetry):
    figure = make_subplots(
        rows=4,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.065,
        subplot_titles=(
            "ArduRover motor outputs (PWM)",
            "Boat forward speed",
            "Boat yaw and ArduRover estimate",
            "North-East trajectory",
        ),
    )
    time_exchange = values(exchange, "time_s")
    figure.add_trace(go.Scatter(
        x=time_exchange, y=values(exchange, "left_pwm"), name="Left motor",
        line={"color": BLUE, "width": 2},
    ), row=1, col=1)
    figure.add_trace(go.Scatter(
        x=time_exchange, y=values(exchange, "right_pwm"), name="Right motor",
        line={"color": RED, "width": 2},
    ), row=1, col=1)
    figure.add_trace(go.Scatter(
        x=values(truth, "time_s"), y=values(truth, "u_mps"),
        name="Plant forward speed", line={"color": BLUE, "width": 2},
    ), row=2, col=1)
    figure.add_trace(go.Scatter(
        x=values(telemetry, "time_s"), y=values(telemetry, "groundspeed_mps"),
        name="ArduRover groundspeed", line={"color": ORANGE, "width": 2},
    ), row=2, col=1)
    figure.add_trace(go.Scatter(
        x=values(truth, "time_s"), y=values(truth, "yaw_deg"),
        name="Plant yaw", line={"color": BLUE, "width": 2},
    ), row=3, col=1)
    figure.add_trace(go.Scatter(
        x=values(telemetry, "time_s"), y=values(telemetry, "yaw_est_deg"),
        name="ArduRover yaw estimate", line={"color": ORANGE, "width": 2},
    ), row=3, col=1)
    figure.add_trace(go.Scatter(
        x=values(truth, "east_m"), y=values(truth, "north_m"),
        name="Plant path", mode="lines", line={"color": BLUE, "width": 3},
    ), row=4, col=1)
    figure.update_xaxes(title_text="Simulation time (s)", row=3, col=1)
    figure.update_xaxes(title_text="East (m)", row=4, col=1)
    figure.update_yaxes(title_text="PWM", row=1, col=1)
    figure.update_yaxes(title_text="m/s", row=2, col=1)
    figure.update_yaxes(title_text="degrees", row=3, col=1)
    figure.update_yaxes(title_text="North (m)", scaleanchor="x4", row=4, col=1)
    figure.update_layout(
        height=1250,
        margin={"l": 80, "r": 40, "t": 80, "b": 60},
        paper_bgcolor="#f8fafc",
        plot_bgcolor="#ffffff",
        font={"family": "Inter, system-ui, sans-serif", "color": "#172033"},
        hovermode="x unified",
        legend={"orientation": "h", "y": 1.04, "x": 1.0, "xanchor": "right"},
    )
    return pio.to_html(figure, include_plotlyjs=True, full_html=False)


def main(run_dir):
    truth = read_rows(run_dir / "truth.csv")
    exchange = read_rows(run_dir / "exchange.csv")
    telemetry = read_rows(run_dir / "telemetry.csv")
    if not truth or not telemetry:
        raise RuntimeError("boat run did not produce truth and telemetry")
    timing = lockstep_summary(exchange)
    forward = phase_rows(telemetry, "forward")
    turn = phase_rows(telemetry, "turn-right")
    stop = phase_rows(telemetry, "stop")
    turn_start = min(float(row["time_s"]) for row in turn)
    turn_end = max(float(row["time_s"]) for row in turn)
    turn_truth = [
        row for row in truth if turn_start <= float(row["time_s"]) <= turn_end
    ]
    final_truth = [row for row in truth if float(row["time_s"]) >= 27.0]
    finite = all(
        math.isfinite(float(value))
        for row in truth
        for key, value in row.items()
        if key != "time_s"
    )
    maximum_distance = max(
        math.hypot(float(row["north_m"]), float(row["east_m"])) for row in truth
    )
    forward_speed = max(float(row["groundspeed_mps"]) for row in forward)
    turn_yaw_rate = median(turn_truth, "r_radps")
    left_turn_pwm = median(
        [row for row in exchange if turn_start <= float(row["time_s"]) <= turn_end],
        "left_pwm",
    )
    right_turn_pwm = median(
        [row for row in exchange if turn_start <= float(row["time_s"]) <= turn_end],
        "right_pwm",
    )
    final_speed = median(final_truth, "u_mps")
    checks = {
        "Controller exchange remained in lockstep": timing["verified"],
        "All plant values remained finite": finite,
        "Run remained within 100 m of its origin": maximum_distance < 100.0,
        "Forward phase exceeded 0.25 m/s": forward_speed > 0.25,
        "Right-turn phase produced positive yaw rate": turn_yaw_rate > math.radians(2.0),
        "Right-turn mixer drove left above right": left_turn_pwm > right_turn_pwm,
        "Stop phase returned below 0.25 m/s": abs(final_speed) < 0.25,
    }
    passed = all(checks.values())
    cards = [
        ("Lockstep", "Verified" if timing["verified"] else "Failed"),
        ("Controller exchange", f"{timing['frames']} @ {timing['rate_hz']} Hz"),
        ("Frame faults", f"{timing['missed']} missed · {timing['duplicates']} duplicate"),
        ("Maximum distance", f"{maximum_distance:.2f} m"),
        ("Forward speed", f"{forward_speed:.2f} m/s"),
        ("Turn yaw rate", f"{math.degrees(turn_yaw_rate):.2f}°/s"),
    ]
    card_html = "".join(
        f"<div class='card'><span>{html.escape(label)}</span><strong>{html.escape(value)}</strong></div>"
        for label, value in cards
    )
    check_html = "".join(
        f"<li class='{'pass' if ok else 'fail'}'>{'✓' if ok else '✗'} {html.escape(label)}</li>"
        for label, ok in checks.items()
    )
    document = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>UVD ArduRover Otter run</title>
<style>
body{{margin:0;background:#f1f5f9;color:#172033;font:16px Inter,system-ui,sans-serif}}
main{{max-width:1500px;margin:0 auto;padding:32px}}
h1{{margin:0 0 8px}} .summary{{color:#64748b;margin-bottom:24px}}
.cards{{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin-bottom:20px}}
.card{{background:white;border:1px solid #dbe3ee;border-radius:12px;padding:18px}}
.card span{{display:block;color:#64748b}} .card strong{{display:block;font-size:25px;margin-top:5px}}
.checks{{background:white;border:1px solid #dbe3ee;border-radius:12px;padding:18px 26px}}
.checks li{{list-style:none;margin:7px 0}} .pass{{color:#15803d}} .fail{{color:#b91c1c}}
</style></head><body><main><h1>ArduRover + Fossen Otter</h1>
<p class="summary">Bounded ACRO speed-and-turn demonstration · {'PASS' if passed else 'CHECK FAILED'}</p>
<section class="cards">{card_html}</section><ul class="checks">{check_html}</ul>
{make_plot(truth, exchange, telemetry)}</main></body></html>"""
    (run_dir / "report.html").write_text(document)
    if not passed:
        failed = "; ".join(label for label, ok in checks.items() if not ok)
        raise RuntimeError(f"ArduRover demonstration failed: {failed}")


if __name__ == "__main__":
    try:
        main(Path(sys.argv[1] if len(sys.argv) > 1 else "/run"))
    except (OSError, RuntimeError, ValueError) as error:
        print(f"report: {error}", file=sys.stderr)
        sys.exit(1)
