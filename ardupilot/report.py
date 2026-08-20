#!/usr/bin/env python3
"""Render the bounded FBWA roll run as a self-contained Plotly report."""

import bisect
import csv
import html
from pathlib import Path
import statistics
import sys

import plotly.graph_objects as go
import plotly.io as pio
from plotly.subplots import make_subplots


BLUE = "#2563eb"
GREEN = "#16a34a"
RED = "#dc2626"
SLATE = "#64748b"
PURPLE = "#7c3aed"
ORANGE = "#ea580c"


def read_rows(path):
    with path.open(newline="") as source:
        return list(csv.DictReader(source))


def points(rows, field):
    result = []
    for row in rows:
        point = (float(row["time_s"]), float(row[field]))
        if result and point[0] == result[-1][0]:
            result[-1] = point
        else:
            result.append(point)
    return result


def interpolate(series, time_s):
    times = [point[0] for point in series]
    index = bisect.bisect_left(times, time_s)
    if index == 0:
        return series[0][1]
    if index == len(series):
        return series[-1][1]
    left, right = series[index - 1], series[index]
    fraction = (time_s - left[0]) / (right[0] - left[0])
    return left[1] + fraction * (right[1] - left[1])


def angle_difference(left, right):
    return abs((left - right + 180.0) % 360.0 - 180.0)


def maximum_difference(reference, comparison, wrap=False):
    start = comparison[0][0]
    end = comparison[-1][0]
    difference = angle_difference if wrap else lambda left, right: abs(left - right)
    return max(
        difference(value, interpolate(comparison, time_s))
        for time_s, value in reference
        if start <= time_s <= end
    )


def lockstep_summary(exchange):
    if len(exchange) < 2:
        raise RuntimeError("exchange log does not contain enough frames")
    frames = [int(row["servo_frame"]) for row in exchange]
    steps = [int(row["step"]) for row in exchange]
    physics_times = [float(row["physics_time_s"]) for row in exchange]
    rates = {int(row["frame_rate_hz"]) for row in exchange}
    frame_gaps = [right - left for left, right in zip(frames, frames[1:])]
    step_gaps = [right - left for left, right in zip(steps, steps[1:])]
    time_steps = [
        right - left for left, right in zip(physics_times, physics_times[1:])
    ]
    rate_hz = next(iter(rates)) if len(rates) == 1 else 0
    fixed_dt_s = statistics.median(time_steps)
    missed = max(int(row["missed_frames"]) for row in exchange)
    duplicates = max(int(row["duplicate_frames"]) for row in exchange)
    rate_matches = rate_hz > 0 and abs(fixed_dt_s - 1.0 / rate_hz) < 1.0e-6
    verified = (
        all(gap == 1 for gap in frame_gaps)
        and all(gap == 1 for gap in step_gaps)
        and missed == 0
        and duplicates == 0
        and rate_matches
    )
    return {
        "verified": verified,
        "frames": len(exchange),
        "rate_hz": rate_hz,
        "fixed_dt_s": fixed_dt_s,
        "missed": missed,
        "duplicates": duplicates,
    }


def state_match_score(received, sent):
    differences = (
        angle_difference(
            float(received["roll_received_deg"]), float(sent["roll_deg"])
        ),
        angle_difference(
            float(received["pitch_received_deg"]), float(sent["pitch_deg"])
        ),
        angle_difference(
            float(received["yaw_received_deg"]), float(sent["yaw_deg"])
        ),
        10.0 * (float(received["ax_received_mps2"]) - float(sent["ax_mps2"])),
        10.0 * (float(received["ay_received_mps2"]) - float(sent["ay_mps2"])),
        10.0 * (float(received["az_received_mps2"]) - float(sent["az_mps2"])),
        50.0 * (float(received["p_received_radps"]) - float(sent["p_radps"])),
        50.0 * (float(received["q_received_radps"]) - float(sent["q_radps"])),
        50.0 * (float(received["r_received_radps"]) - float(sent["r_radps"])),
    )
    return sum(value * value for value in differences)


def align_received_state(simstate, exchange):
    """Match timestamp-less MAVLink SIMSTATE samples to exact JSON frames."""
    aligned = []
    attitude_errors = []
    cursor = 0
    for received in simstate:
        match = min(
            range(cursor, len(exchange)),
            key=lambda index: state_match_score(received, exchange[index]),
        )
        if state_match_score(received, exchange[match]) > 1.0e-6:
            continue
        cursor = match
        row = dict(received)
        row["time_s"] = exchange[match]["time_s"]
        aligned.append(row)
        attitude_errors.extend(
            angle_difference(
                float(received[f"{axis}_received_deg"]),
                float(exchange[match][f"{axis}_deg"]),
            )
            for axis in ("roll", "pitch", "yaw")
        )
    if len(aligned) < len(simstate) - 2:
        raise RuntimeError("ArduPilot SIMSTATE did not match the sent plant states")
    return aligned, max(attitude_errors)


def trace(series, name, color, hover, suffix="", dash=None, **options):
    line = {"color": color, "width": 2}
    if dash:
        line["dash"] = dash
    return go.Scatter(
        x=[point[0] for point in series],
        y=[point[1] for point in series],
        mode="lines",
        name=name,
        line=line,
        hovertemplate=(
            f"%{{x:.3f}} s · %{{y:{hover}}}{suffix}<extra>{name}</extra>"
        ),
        **options,
    )


def sample_trace(series, name, color, hover, suffix="", **options):
    visible = [series[0]]
    next_time_s = series[0][0] + 0.5
    for point in series[1:-1]:
        if point[0] + 1.0e-9 >= next_time_s:
            visible.append(point)
            next_time_s += 0.5
    if series[-1] != visible[-1]:
        visible.append(series[-1])
    return go.Scatter(
        x=[point[0] for point in visible],
        y=[point[1] for point in visible],
        mode="markers",
        name=name,
        marker={
            "color": color,
            "opacity": 0.75,
            "size": 5,
            "symbol": "circle-open",
            "line": {"color": color, "width": 1.25},
        },
        hovertemplate=(
            f"%{{x:.3f}} s · %{{y:{hover}}}{suffix}<extra>{name}</extra>"
        ),
        **options,
    )


def attitude_traces(figure, row, plant, received, estimate, show_legend):
    figure.add_trace(
        trace(
            plant,
            "Unreal plant state",
            BLUE,
            ".2f",
            "°",
            legendgroup="plant",
            showlegend=show_legend,
        ),
        row=row,
        col=1,
    )
    figure.add_trace(
        sample_trace(
            received,
            "ArduPilot received state",
            GREEN,
            ".2f",
            "°",
            legendgroup="received",
            showlegend=show_legend,
        ),
        row=row,
        col=1,
    )
    figure.add_trace(
        trace(
            estimate,
            "ArduPlane estimate",
            RED,
            ".2f",
            "°",
            legendgroup="estimate",
            showlegend=show_legend,
        ),
        row=row,
        col=1,
    )


def plot_html(command, plant, received, estimate, outputs):
    figure = make_subplots(
        rows=5,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.055,
        row_heights=[0.15, 0.22, 0.22, 0.21, 0.20],
        subplot_titles=(
            "Pilot roll input (PWM)",
            "Roll attitude (degrees)",
            "Pitch attitude (degrees)",
            "Yaw attitude (degrees)",
            "ArduPlane controller outputs (PWM)",
        ),
    )
    roll_input = trace(command, "Pilot roll input", SLATE, ".0f", showlegend=False)
    roll_input.line.shape = "hv"
    figure.add_trace(roll_input, row=1, col=1)
    attitude_traces(
        figure, 2, plant["roll"], received["roll"], estimate["roll"], True
    )
    attitude_traces(
        figure, 3, plant["pitch"], received["pitch"], estimate["pitch"], False
    )
    attitude_traces(
        figure, 4, plant["yaw"], received["yaw"], estimate["yaw"], False
    )
    figure.add_trace(
        trace(outputs["aileron"], "Aileron output", PURPLE, ".0f"),
        row=5,
        col=1,
    )
    figure.add_trace(
        trace(outputs["elevator"], "Elevator output", ORANGE, ".0f"),
        row=5,
        col=1,
    )
    figure.update_layout(
        height=1180,
        margin={"l": 68, "r": 24, "t": 76, "b": 58},
        paper_bgcolor="white",
        plot_bgcolor="white",
        font={
            "family": "system-ui, -apple-system, BlinkMacSystemFont, sans-serif",
            "color": "#172033",
        },
        hovermode="x unified",
        legend={
            "orientation": "h",
            "x": 1,
            "xanchor": "right",
            "y": 1.06,
            "yanchor": "top",
        },
    )
    figure.update_annotations(font={"size": 16}, x=0, xanchor="left")
    figure.update_xaxes(
        range=[0, 12],
        gridcolor="#e7ebf0",
        linecolor="#94a3b8",
        showline=True,
        zeroline=False,
        ticksuffix=" s",
    )
    figure.update_yaxes(
        gridcolor="#e7ebf0",
        linecolor="#94a3b8",
        showline=True,
        zeroline=False,
    )
    figure.update_yaxes(range=[1250, 1750], row=1, col=1)
    figure.update_yaxes(range=[1000, 2000], row=5, col=1)
    figure.update_xaxes(title_text="Simulation time", row=5, col=1)
    for boundary in (2, 5, 8):
        for row in range(1, 6):
            figure.add_vline(
                x=boundary,
                line={"color": "#cbd5e1", "dash": "dot", "width": 1},
                row=row,
                col=1,
            )
    return pio.to_html(
        figure,
        include_plotlyjs=True,
        full_html=False,
        div_id="uvd-plots",
        config={
            "displaylogo": False,
            "responsive": True,
            "scrollZoom": True,
            "toImageButtonOptions": {"filename": "uvd-fbwa-roll"},
        },
    )


def main(run_dir):
    run_dir = Path(run_dir).resolve()
    truth = read_rows(run_dir / "truth.csv")
    telemetry = read_rows(run_dir / "telemetry.csv")
    exchange = read_rows(run_dir / "exchange.csv")
    simstate = read_rows(run_dir / "simstate.csv")
    bins = sorted((run_dir / "ardupilot" / "logs").glob("*.BIN"))
    if not truth or not telemetry or not exchange or not simstate or not bins:
        raise RuntimeError(
            "run is missing plant, exchange, SIMSTATE, telemetry, or DataFlash data"
        )

    plant = {
        axis: points(truth, f"{axis}_deg") for axis in ("roll", "pitch", "yaw")
    }
    aligned_simstate, received_error = align_received_state(simstate, exchange)
    received = {
        axis: points(aligned_simstate, f"{axis}_received_deg")
        for axis in ("roll", "pitch", "yaw")
    }
    estimate = {
        axis: points(telemetry, f"{axis}_est_deg")
        for axis in ("roll", "pitch", "yaw")
    }
    command = points(telemetry, "roll_pwm")
    outputs = {
        "aileron": points(exchange, "aileron_pwm"),
        "elevator": points(exchange, "elevator_pwm"),
    }
    roll_error = maximum_difference(plant["roll"], estimate["roll"])
    pitch_error = maximum_difference(plant["pitch"], estimate["pitch"])
    lockstep = lockstep_summary(exchange)
    plots = plot_html(command, plant, received, estimate, outputs)
    bin_links = " · ".join(
        f'<a href="{html.escape(path.relative_to(run_dir).as_posix())}">'
        f"{html.escape(path.name)}</a>"
        for path in bins
    )
    status = "Verified" if lockstep["verified"] else "Failed"
    status_class = "pass" if lockstep["verified"] else "fail"
    document = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>UVD FBWA roll run</title>
<style>
body{{font:15px system-ui,sans-serif;color:#172033;background:#f5f7fa;margin:0}}
main{{max-width:1180px;margin:32px auto;padding:0 20px}} h1{{margin-bottom:4px}}
.muted{{color:#64748b}} .summary{{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin:24px 0}}
.card,.plot{{background:white;border:1px solid #dbe2ea;border-radius:10px}} .card{{padding:14px}}
.value{{font-size:24px;font-weight:650;margin-top:4px}} .pass{{color:#15803d}} .fail{{color:#b91c1c}}
.plot{{overflow:hidden;padding:4px}} a{{color:#2563eb}}
@media(max-width:760px){{.summary{{grid-template-columns:repeat(2,1fr)}}}}
</style></head><body><main>
<h1>FBWA roll run</h1><div class="muted">Aerosonde · ArduPlane · Unreal/Cesium/Chaos</div>
<section class="summary">
<div class="card">Lockstep<div class="value {status_class}">{status}</div></div>
<div class="card">Controller exchange<div class="value">{lockstep['frames']} @ {lockstep['rate_hz']} Hz</div></div>
<div class="card">Frame faults<div class="value">{lockstep['missed']} missed · {lockstep['duplicates']} duplicate</div></div>
<div class="card">State transfer<div class="value">{received_error:.4f}° max</div></div>
<div class="card">Max roll estimate difference<div class="value">{roll_error:.2f}°</div></div>
<div class="card">Max pitch estimate difference<div class="value">{pitch_error:.2f}°</div></div>
</section>
<p>Blue is the Unreal plant pose, green circles are samples of the same state after ArduPilot parsed the JSON input, and red is ArduPlane's attitude estimate. SIMSTATE samples are aligned to the exact controller exchange frame they match.</p>
<section class="plot">{plots}</section>
<p class="muted">Lockstep requires consecutive controller frames and physics steps, matching frame rates, and no missed or duplicate frames.</p>
<p>ArduPilot DataFlash: {bin_links}</p>
</main></body></html>"""
    output = run_dir / "report.html"
    output.write_text(document)
    print(output)
    if not lockstep["verified"]:
        raise RuntimeError("controller exchange did not remain in lockstep")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: report.py RUN_DIRECTORY")
    try:
        main(sys.argv[1])
    except (OSError, ValueError, RuntimeError) as error:
        print(f"report: {error}", file=sys.stderr)
        raise SystemExit(1)
