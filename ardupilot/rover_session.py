#!/usr/bin/env python3
"""Run one bounded ArduRover ACRO speed-and-turn demonstration."""

import csv
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

from pymavlink import mavutil


running = True


def stop(_signal, _frame):
    global running
    running = False


def release_unreal(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control:
        for _ in range(5):
            control.sendto(b"release\n", (socket.gethostbyname(host), port))
            time.sleep(0.05)


def rc_override(connection, steering_pwm, throttle_pwm):
    ignore = 65535
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        steering_pwm,
        1500,
        throttle_pwm,
        1500,
        ignore,
        ignore,
        ignore,
        ignore,
    )


def request_message(connection, message_id, rate_hz):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        1_000_000 / rate_hz,
        0,
        0,
        0,
        0,
        0,
    )


def command(elapsed_s):
    if elapsed_s < 2.0:
        return "settle", 1500, 1500
    if elapsed_s < 8.0:
        return "forward", 1500, 1700
    if elapsed_s < 14.0:
        return "turn-right", 1650, 1700
    if elapsed_s < 18.0:
        return "recover", 1500, 1700
    return "stop", 1500, 1500


def main():
    host = os.environ["UVD_HOST_ADDRESS"]
    sim_address = os.environ["UVD_SIM_ADDRESS"]
    connection = mavutil.mavlink_connection(
        "udpin:0.0.0.0:14550", source_system=255, autoreconnect=False
    )
    run_dir = Path(os.environ.get("UVD_RUN_DIR", "/run"))
    ready_path = (
        Path(os.environ["UVD_READY_FILE"]) if "UVD_READY_FILE" in os.environ else None
    )
    start_path = (
        Path(os.environ["UVD_START_FILE"]) if "UVD_START_FILE" in os.environ else None
    )
    done_path = (
        Path(os.environ["UVD_DONE_FILE"]) if "UVD_DONE_FILE" in os.environ else None
    )
    finish_path = (
        Path(os.environ["UVD_FINISH_FILE"]) if "UVD_FINISH_FILE" in os.environ else None
    )
    duration_s = float(os.environ.get("UVD_DURATION_S", "28.0"))
    ardupilot_dir = run_dir / "ardupilot"
    ardupilot_dir.mkdir(parents=True, exist_ok=True)
    telemetry_file = (run_dir / "telemetry.csv").open("w", newline="")
    telemetry = csv.writer(telemetry_file)
    telemetry.writerow(
        [
            "time_s",
            "phase",
            "steering_pwm",
            "throttle_pwm",
            "mode",
            "armed",
            "roll_est_deg",
            "pitch_est_deg",
            "yaw_est_deg",
            "groundspeed_mps",
            "speed_target_mps",
            "speed_estimate_mps",
            "yaw_rate_target_degps",
            "yaw_rate_estimate_degps",
        ]
    )
    telemetry_file.flush()
    rover = subprocess.Popen(
        [
            "/opt/ardupilot/ardurover",
            "--model",
            f"JSON:{sim_address}",
            "--home",
            os.environ["UVD_HOME"],
            "--speedup",
            "1",
            "--rate",
            os.environ.get("UVD_RATE_HZ", "120"),
            "--sim-port-in",
            os.environ.get("UVD_SIM_PORT_IN", "9003"),
            "--sim-port-out",
            os.environ.get("UVD_SIM_PORT_OUT", "9002"),
            "--sysid",
            os.environ.get("UVD_SYSID", "1"),
            "--serial0",
            "udpclient:127.0.0.1:14550",
            "--defaults",
            "/opt/ardupilot/rover.parm",
        ],
        cwd=ardupilot_dir,
    )

    mode = ""
    armed = False
    latest_attitude = None
    groundspeed_mps = 0.0
    speed_target_mps = math.nan
    speed_estimate_mps = math.nan
    yaw_rate_target_degps = math.nan
    yaw_rate_estimate_degps = math.nan
    released = False
    release_boot_ms = None
    elapsed_s = 0.0
    phase = "preflight"
    steering_pwm = 1500
    throttle_pwm = 1500
    completed = False
    streams_requested = False
    last_request = 0.0
    try:
        while running and rover.poll() is None:
            message = connection.recv_match(blocking=True, timeout=0.1)
            if message:
                kind = message.get_type()
                if kind == "HEARTBEAT" and message.type != mavutil.mavlink.MAV_TYPE_GCS:
                    connection.target_system = message.get_srcSystem()
                    connection.target_component = message.get_srcComponent()
                    mode = mavutil.mode_string_v10(message)
                    armed = bool(
                        message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                elif kind == "VFR_HUD":
                    groundspeed_mps = float(message.groundspeed)
                elif kind == "PID_TUNING":
                    # Rover uses the generic ACCZ enum for its speed loop and
                    # STEER for its yaw-rate loop. GCS_PID_MASK=3 enables both;
                    # only target and feedback are needed for this small demo.
                    if int(message.axis) == mavutil.mavlink.PID_TUNING_ACCZ:
                        speed_target_mps = float(message.desired)
                        speed_estimate_mps = float(message.achieved)
                    elif int(message.axis) == mavutil.mavlink.PID_TUNING_STEER:
                        yaw_rate_target_degps = float(message.desired)
                        yaw_rate_estimate_degps = float(message.achieved)
                elif kind == "ATTITUDE":
                    latest_attitude = message
                    if released:
                        elapsed_s = (
                            int(message.time_boot_ms) - release_boot_ms
                        ) / 1000.0
                        reached_completion = (
                            elapsed_s >= duration_s + 0.2 and not completed
                        )
                        next_command = command(elapsed_s)
                        if next_command != (phase, steering_pwm, throttle_pwm):
                            phase, steering_pwm, throttle_pwm = next_command
                            rc_override(connection, steering_pwm, throttle_pwm)
                        telemetry.writerow(
                            [
                                f"{elapsed_s:.3f}",
                                phase,
                                steering_pwm,
                                throttle_pwm,
                                mode,
                                int(armed),
                                f"{math.degrees(message.roll):.6f}",
                                f"{math.degrees(message.pitch):.6f}",
                                f"{math.degrees(message.yaw):.6f}",
                                f"{groundspeed_mps:.6f}",
                                f"{speed_target_mps:.6f}",
                                f"{speed_estimate_mps:.6f}",
                                f"{yaw_rate_target_degps:.6f}",
                                f"{yaw_rate_estimate_degps:.6f}",
                            ]
                        )
                        telemetry_file.flush()
                        if reached_completion:
                            completed = True
                            if done_path:
                                done_path.write_text("done\n")
                                if finish_path is None:
                                    break
                            else:
                                break

            now = time.monotonic()
            if connection.target_system and not streams_requested:
                request_message(connection, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
                request_message(connection, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 10)
                request_message(
                    connection, mavutil.mavlink.MAVLINK_MSG_ID_PID_TUNING, 20
                )
                streams_requested = True
            if connection.target_system and now - last_request >= 0.5:
                rc_override(connection, steering_pwm, throttle_pwm)
                # ACRO exercises Rover's speed and turn-rate controllers. HOLD
                # is selected only after the maneuver so the stop phase has an
                # unambiguous zero-motor command instead of pilot-stick mapping.
                requested_mode = "HOLD" if released and elapsed_s >= 18.0 else "ACRO"
                if mode != requested_mode:
                    mode_id = connection.mode_mapping().get(requested_mode)
                    if mode_id is not None:
                        connection.mav.set_mode_send(
                            connection.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            mode_id,
                        )
                elif requested_mode == "ACRO" and not armed:
                    # 21196 is ArduPilot's explicit SITL force-arm value. The
                    # demo still records mode and arm state before release.
                    connection.mav.command_long_send(
                        connection.target_system,
                        connection.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        1.0,
                        21196.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    )
                elif (
                    requested_mode == "ACRO"
                    and not released
                    and latest_attitude is not None
                ):
                    if ready_path and not ready_path.exists():
                        ready_path.write_text("ready\n")
                        print(
                            "ArduRover is armed in ACRO; waiting for scenario start",
                            flush=True,
                        )
                    if not start_path or start_path.exists():
                        release_boot_ms = int(latest_attitude.time_boot_ms)
                        phase, steering_pwm, throttle_pwm = command(0.0)
                        rc_override(connection, steering_pwm, throttle_pwm)
                        release_unreal(
                            host,
                            int(os.environ.get("UVD_CONTROL_PORT", "9003")),
                        )
                        print(
                            "ArduRover is armed in ACRO; releasing vehicle dynamics",
                            flush=True,
                        )
                        released = True
                last_request = now
            if completed and finish_path and finish_path.is_file():
                break
    finally:
        if rover.poll() is None:
            rover.terminate()
            try:
                rover.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rover.kill()
                rover.wait()
        if completed:
            (ardupilot_dir / "eeprom.bin").unlink(missing_ok=True)
            (ardupilot_dir / "logs" / "LASTLOG.TXT").unlink(missing_ok=True)
            shutil.rmtree(ardupilot_dir / "terrain", ignore_errors=True)
        telemetry_file.close()
    return 0 if completed else (rover.returncode or 1)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    sys.exit(main())
