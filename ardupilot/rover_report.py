#!/usr/bin/env python3
"""Validate and plot the bounded ArduRover ACRO boat run."""

import html
import math
from pathlib import Path
import sys

import plotly.graph_objects as go
import plotly.io as pio
from plotly.subplots import make_subplots

from report_common import lockstep_summary, read_rows


BLUE = "#2563eb"
RED = "#dc2626"
SLATE = "#64748b"
ORANGE = "#ea580c"


def values(rows, field):
    return [float(row[field]) for row in rows]


def rms_difference(rows, target, actual):
    return math.sqrt(
        sum((float(row[target]) - float(row[actual])) ** 2 for row in rows) / len(rows)
    )


def saturation_percent(rows, field):
    saturated = sum(
        float(row[field]) <= 1000.0 or float(row[field]) >= 2000.0 for row in rows
    )
    return 100.0 * saturated / len(rows)


def make_plot(truth, exchange, telemetry):
    figure = make_subplots(
        rows=4,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.065,
        subplot_titles=(
            "Speed response",
            "Yaw-rate response",
            "ArduRover motor outputs",
            "North-East trajectory",
        ),
    )
    active = [
        row
        for row in telemetry
        if row["phase"] in {"forward", "turn-right", "recover"}
        and row["mode"] == "ACRO"
        and all(
            math.isfinite(float(row[field]))
            for field in (
                "speed_target_mps",
                "speed_estimate_mps",
                "yaw_rate_target_degps",
                "yaw_rate_estimate_degps",
            )
        )
    ]
    active_time = values(active, "time_s")
    truth_time = values(truth, "time_s")
    figure.add_trace(
        go.Scatter(
            x=active_time,
            y=values(active, "speed_target_mps"),
            name="Speed target",
            line={"color": SLATE, "width": 2, "dash": "dash"},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=active_time,
            y=values(active, "speed_estimate_mps"),
            name="ArduRover speed",
            line={"color": ORANGE, "width": 2},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=truth_time,
            y=values(truth, "u_mps"),
            name="Plant forward speed",
            line={"color": BLUE, "width": 2},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=active_time,
            y=values(active, "yaw_rate_target_degps"),
            name="Yaw-rate target",
            line={"color": SLATE, "width": 2, "dash": "dash"},
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=active_time,
            y=values(active, "yaw_rate_estimate_degps"),
            name="ArduRover yaw rate",
            line={"color": ORANGE, "width": 2},
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=truth_time,
            y=[math.degrees(value) for value in values(truth, "r_radps")],
            name="Plant yaw rate",
            line={"color": BLUE, "width": 2},
        ),
        row=2,
        col=1,
    )
    time_exchange = values(exchange, "time_s")
    figure.add_trace(
        go.Scatter(
            x=time_exchange,
            y=values(exchange, "left_pwm"),
            name="Left motor",
            line={"color": BLUE, "width": 2},
        ),
        row=3,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=time_exchange,
            y=values(exchange, "right_pwm"),
            name="Right motor",
            line={"color": RED, "width": 2},
        ),
        row=3,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=values(truth, "east_m"),
            y=values(truth, "north_m"),
            name="Plant path",
            mode="lines",
            line={"color": BLUE, "width": 3},
        ),
        row=4,
        col=1,
    )
    for limit in (1000, 2000):
        figure.add_hline(
            y=limit,
            row=3,
            col=1,
            line={"color": "#94a3b8", "width": 1, "dash": "dot"},
        )
    for boundary in (2, 8, 14, 18):
        for row in range(1, 4):
            figure.add_vline(
                x=boundary,
                row=row,
                col=1,
                line={"color": "#cbd5e1", "width": 1, "dash": "dot"},
            )
    figure.update_xaxes(title_text="Simulation time (s)", row=3, col=1)
    figure.update_xaxes(title_text="East (m)", row=4, col=1)
    figure.update_yaxes(title_text="m/s", row=1, col=1)
    figure.update_yaxes(title_text="deg/s", row=2, col=1)
    figure.update_yaxes(title_text="PWM", range=[950, 2050], row=3, col=1)
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
    if not truth or not exchange or not telemetry:
        raise RuntimeError("boat run did not produce truth, exchange, and telemetry")
    timing = lockstep_summary(exchange)
    phases = {row["phase"] for row in telemetry}
    phases_complete = {"settle", "forward", "turn-right", "recover", "stop"}.issubset(
        phases
    )
    finite = all(
        math.isfinite(float(value))
        for row in truth
        for key, value in row.items()
        if key != "time_s"
    )
    active = [
        row
        for row in telemetry
        if row["phase"] in {"forward", "turn-right", "recover"}
        and row["mode"] == "ACRO"
        and all(
            math.isfinite(float(row[field]))
            for field in (
                "speed_target_mps",
                "speed_estimate_mps",
                "yaw_rate_target_degps",
                "yaw_rate_estimate_degps",
            )
        )
    ]
    controller_signals_present = bool(active)
    active_exchange = [row for row in exchange if 2.0 <= float(row["time_s"]) < 18.0]
    if not active_exchange:
        raise RuntimeError("exchange log is missing the active maneuver")
    speed_rms = (
        rms_difference(active, "speed_target_mps", "speed_estimate_mps")
        if active
        else math.nan
    )
    yaw_rate_rms = (
        rms_difference(active, "yaw_rate_target_degps", "yaw_rate_estimate_degps")
        if active
        else math.nan
    )
    left_saturation = saturation_percent(active_exchange, "left_pwm")
    right_saturation = saturation_percent(active_exchange, "right_pwm")
    checks = {
        "Controller exchange remained in lockstep": timing["verified"],
        "All plant values remained finite": finite,
        "All maneuver phases were recorded": phases_complete,
        "Speed and yaw-rate controller signals were recorded": controller_signals_present,
    }
    complete = all(checks.values())
    cards = [
        ("Lockstep", "Verified" if timing["verified"] else "Failed"),
        ("Controller exchange", f"{timing['frames']} @ {timing['rate_hz']} Hz"),
        (
            "Frame faults",
            f"{timing['missed']} missed · {timing['duplicates']} duplicate",
        ),
        ("Speed RMS difference", f"{speed_rms:.2f} m/s"),
        ("Yaw-rate RMS difference", f"{yaw_rate_rms:.2f}°/s"),
        ("Left motor saturation", f"{left_saturation:.1f}%"),
        ("Right motor saturation", f"{right_saturation:.1f}%"),
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
.cards{{display:grid;grid-template-columns:repeat(4,1fr);gap:14px;margin-bottom:20px}}
.card{{background:white;border:1px solid #dbe3ee;border-radius:12px;padding:18px}}
.card span{{display:block;color:#64748b}} .card strong{{display:block;font-size:25px;margin-top:5px}}
.checks{{background:white;border:1px solid #dbe3ee;border-radius:12px;padding:18px 26px}}
.checks li{{list-style:none;margin:7px 0}} .pass{{color:#15803d}} .fail{{color:#b91c1c}}
@media(max-width:900px){{.cards{{grid-template-columns:repeat(2,1fr)}}}}
</style></head><body><main><h1>ArduRover + Fossen Otter</h1>
<p class="summary">Bounded ACRO speed-and-turn demonstration · {'RUN COMPLETE' if complete else 'RUN INCOMPLETE'}</p>
<section class="cards">{card_html}</section><ul class="checks">{check_html}</ul>
<p class="summary">Tracking differences and saturation are measurements for this run, not universal pass/fail limits.</p>
{make_plot(truth, exchange, telemetry)}</main></body></html>"""
    (run_dir / "report.html").write_text(document)
    if not complete:
        failed = "; ".join(label for label, ok in checks.items() if not ok)
        raise RuntimeError(f"ArduRover demonstration incomplete: {failed}")


if __name__ == "__main__":
    try:
        main(Path(sys.argv[1] if len(sys.argv) > 1 else "/run"))
    except (OSError, RuntimeError, ValueError) as error:
        print(f"report: {error}", file=sys.stderr)
        sys.exit(1)
