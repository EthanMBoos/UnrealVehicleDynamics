#!/usr/bin/env python3
"""Validate and plot one synchronized Plane and Otter scenario."""

import html
import json
import math
from pathlib import Path
import sys

import plotly.graph_objects as go
import plotly.io as pio
from plotly.subplots import make_subplots

from report_common import (
    dual_lockstep_summary,
    lockstep_summary,
    read_rows,
    run_coverage_summary,
)


BLUE = "#2563eb"
ORANGE = "#ea580c"
RED = "#dc2626"
SLATE = "#64748b"


def values(rows, field):
    return [float(row[field]) for row in rows]


def finite_rows(rows, ignored=()):
    return all(
        math.isfinite(float(value))
        for row in rows
        for field, value in row.items()
        if field not in ignored
    )


def make_plot(plane_truth, otter_truth, plane_exchange, otter_exchange):
    count = min(len(plane_exchange), len(otter_exchange))
    time_s = values(plane_exchange[:count], "time_s")
    step_skew = [
        int(plane_exchange[index]["step"]) - int(otter_exchange[index]["step"])
        for index in range(count)
    ]
    figure = make_subplots(
        rows=3,
        cols=1,
        vertical_spacing=0.09,
        subplot_titles=(
            "Shared North-East trajectory",
            "Controller outputs",
            "Plane step minus Otter step",
        ),
    )
    figure.add_trace(
        go.Scatter(
            x=values(plane_truth, "east_m"),
            y=values(plane_truth, "north_m"),
            name="Aerosonde",
            line={"color": BLUE, "width": 3},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=values(otter_truth, "east_m"),
            y=values(otter_truth, "north_m"),
            name="Otter",
            line={"color": ORANGE, "width": 3},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=values(plane_exchange, "time_s"),
            y=values(plane_exchange, "aileron_pwm"),
            name="Plane aileron",
            line={"color": BLUE, "width": 2},
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=values(otter_exchange, "time_s"),
            y=values(otter_exchange, "left_pwm"),
            name="Otter left motor",
            line={"color": ORANGE, "width": 2},
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=values(otter_exchange, "time_s"),
            y=values(otter_exchange, "right_pwm"),
            name="Otter right motor",
            line={"color": RED, "width": 2},
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=time_s,
            y=step_skew,
            name="Physics-step skew",
            line={"color": SLATE, "width": 2},
        ),
        row=3,
        col=1,
    )
    figure.update_xaxes(title_text="East (m)", row=1, col=1)
    figure.update_yaxes(
        title_text="North (m)", scaleanchor="x", scaleratio=1, row=1, col=1
    )
    figure.update_xaxes(title_text="Scenario time (s)", row=2, col=1)
    figure.update_yaxes(title_text="PWM", range=[950, 2050], row=2, col=1)
    figure.update_xaxes(title_text="Scenario time (s)", row=3, col=1)
    figure.update_yaxes(title_text="steps", dtick=1, row=3, col=1)
    figure.update_layout(
        height=1100,
        margin={"l": 80, "r": 40, "t": 80, "b": 60},
        paper_bgcolor="#f8fafc",
        plot_bgcolor="#ffffff",
        font={"family": "Inter, system-ui, sans-serif", "color": "#172033"},
        hovermode="closest",
        legend={"orientation": "h", "y": 1.04, "x": 1.0, "xanchor": "right"},
    )
    return pio.to_html(figure, include_plotlyjs=True, full_html=False)


def main(run_dir):
    scenario = json.loads((run_dir / "scenario.json").read_text())
    if scenario.get("format") != "uvd-dual-run-v1":
        raise RuntimeError("dual run manifest has an unsupported format")
    duration_s = float(scenario["duration_s"])
    fixed_dt_s = float(scenario["fixed_dt_s"])
    truth_rate_hz = int(scenario["truth_rate_hz"])
    plane_dir = run_dir / "plane"
    otter_dir = run_dir / "otter"
    plane_truth = read_rows(plane_dir / "truth.csv")
    otter_truth = read_rows(otter_dir / "truth.csv")
    plane_exchange = read_rows(plane_dir / "exchange.csv")
    otter_exchange = read_rows(otter_dir / "exchange.csv")
    if not all((plane_truth, otter_truth, plane_exchange, otter_exchange)):
        raise RuntimeError("dual run is missing plant or controller exchange data")

    plane_timing = lockstep_summary(plane_exchange)
    otter_timing = lockstep_summary(otter_exchange)
    synchronized = dual_lockstep_summary(plane_exchange, otter_exchange)
    plane_coverage = run_coverage_summary(
        plane_exchange, plane_truth, duration_s, fixed_dt_s, truth_rate_hz
    )
    otter_coverage = run_coverage_summary(
        otter_exchange, otter_truth, duration_s, fixed_dt_s, truth_rate_hz
    )
    max_step_skew = synchronized["max_step_skew"]
    max_time_skew = synchronized["max_time_skew_s"]
    clock_tolerance_s = max(1.0e-9, 1.0e-6 * fixed_dt_s)
    checks = {
        "ArduPlane exchange remained in lockstep": plane_timing["verified"],
        "ArduRover exchange remained in lockstep": otter_timing["verified"],
        "Controller exchanges contain the same number of frames": synchronized[
            "same_frame_count"
        ],
        "ArduPlane output covers the configured run": plane_coverage[
            "exchange_complete"
        ]
        and plane_coverage["truth_complete"],
        "ArduRover output covers the configured run": otter_coverage[
            "exchange_complete"
        ]
        and otter_coverage["truth_complete"],
        "Both vehicles advanced on the same physics steps": max_step_skew == 0,
        "Both vehicles used the same physics clock": max_time_skew <= clock_tolerance_s,
        "Both plant truth logs remained finite": finite_rows(plane_truth)
        and finite_rows(otter_truth),
    }
    complete = all(checks.values())
    cards = [
        ("Scenario", "Verified" if complete else "Failed"),
        (
            "Plane exchange",
            f"{plane_timing['frames']} @ {plane_timing['rate_hz']} Hz",
        ),
        (
            "Otter exchange",
            f"{otter_timing['frames']} @ {otter_timing['rate_hz']} Hz",
        ),
        ("Maximum step skew", f"{max_step_skew} steps"),
        ("Maximum clock skew", f"{1.0e6 * max_time_skew:.3f} µs"),
        (
            "Frame faults",
            f"{plane_timing['missed'] + otter_timing['missed']} missed · "
            f"{plane_timing['duplicates'] + otter_timing['duplicates']} duplicate",
        ),
    ]
    card_html = "".join(
        f"<div class='card'><span>{html.escape(label)}</span>"
        f"<strong>{html.escape(value)}</strong></div>"
        for label, value in cards
    )
    check_html = "".join(
        f"<li class='{'pass' if passed else 'fail'}'>"
        f"{'✓' if passed else '✗'} {html.escape(label)}</li>"
        for label, passed in checks.items()
    )
    document = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>UVD dual vehicle run</title>
<style>
body{{margin:0;background:#f1f5f9;color:#172033;font:16px Inter,system-ui,sans-serif}}
main{{max-width:1500px;margin:0 auto;padding:32px}}
h1{{margin:0 0 8px}} a{{color:#2563eb}} .summary{{color:#64748b;margin-bottom:24px}}
.cards{{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin-bottom:20px}}
.card{{background:white;border:1px solid #dbe3ee;border-radius:12px;padding:18px}}
.card span{{display:block;color:#64748b}} .card strong{{display:block;font-size:25px;margin-top:5px}}
.checks{{background:white;border:1px solid #dbe3ee;border-radius:12px;padding:18px 26px}}
.checks li{{list-style:none;margin:7px 0}} .pass{{color:#15803d}} .fail{{color:#b91c1c}}
@media(max-width:900px){{.cards{{grid-template-columns:repeat(2,1fr)}}}}
</style></head><body><main><h1>ArduPlane + ArduRover</h1>
<p class="summary">Aerosonde and Fossen Otter in one Unreal/Cesium world · {'RUN COMPLETE' if complete else 'RUN INCOMPLETE'}</p>
<section class="cards">{card_html}</section><ul class="checks">{check_html}</ul>
<p><a href="plane/report.html">Detailed Plane report</a> · <a href="otter/report.html">Detailed Otter report</a></p>
{make_plot(plane_truth, otter_truth, plane_exchange, otter_exchange)}
</main></body></html>"""
    output = run_dir / "report.html"
    output.write_text(document)
    print(output)
    if not complete:
        failed = "; ".join(label for label, passed in checks.items() if not passed)
        raise RuntimeError(f"dual vehicle demonstration incomplete: {failed}")


if __name__ == "__main__":
    try:
        main(Path(sys.argv[1] if len(sys.argv) > 1 else "/scenario"))
    except (OSError, RuntimeError, ValueError) as error:
        print(f"report: {error}", file=sys.stderr)
        sys.exit(1)
