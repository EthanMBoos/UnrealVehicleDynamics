#!/usr/bin/env python3
"""Run the v1 ArduPlane FBWA roll demonstration."""

import csv
import math
import os
from pathlib import Path
import signal
import shutil
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


def rc_override(connection, roll_pwm, throttle_pwm):
    ignore = 65535
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        roll_pwm,
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


def roll_command(elapsed_s):
    if elapsed_s < 2.0:
        return "settle", 1500
    if elapsed_s < 5.0:
        return "roll-right", 1650
    if elapsed_s < 8.0:
        return "roll-left", 1350
    return "recover", 1500


def main():
    host = os.environ["UVD_HOST_ADDRESS"]
    sim_address = os.environ["UVD_SIM_ADDRESS"]
    connection = mavutil.mavlink_connection(
        "udpin:0.0.0.0:14550", source_system=255, autoreconnect=False
    )
    run_dir = Path(os.environ.get("UVD_RUN_DIR", "/run"))
    ardupilot_dir = run_dir / "ardupilot"
    ardupilot_dir.mkdir(parents=True, exist_ok=True)
    telemetry_file = (run_dir / "telemetry.csv").open("w", newline="")
    telemetry = csv.writer(telemetry_file)
    telemetry.writerow([
        "time_s", "phase", "roll_pwm", "pitch_pwm", "throttle_pwm",
        "rudder_pwm", "mode", "armed", "roll_est_deg", "pitch_est_deg",
        "yaw_est_deg",
    ])
    telemetry_file.flush()
    simstate_file = (run_dir / "simstate.csv").open("w", newline="")
    simstate = csv.writer(simstate_file)
    simstate.writerow([
        "attitude_clock_s", "roll_received_deg", "pitch_received_deg",
        "yaw_received_deg", "ax_received_mps2", "ay_received_mps2",
        "az_received_mps2", "p_received_radps", "q_received_radps",
        "r_received_radps",
    ])
    simstate_file.flush()
    plane = subprocess.Popen(
        [
            "/opt/ardupilot/arduplane",
            "--model", f"JSON:{sim_address}",
            "--home", os.environ["UVD_HOME"],
            "--speedup", "1",
            "--rate", os.environ.get("UVD_RATE_HZ", "120"),
            "--serial0", "udpclient:127.0.0.1:14550",
            "--defaults", "/opt/ardupilot/uvd.parm",
        ],
        cwd=ardupilot_dir,
    )

    mode = ""
    armed = False
    gps_fix = 0
    ekf_flags = 0
    released = False
    release_boot_ms = None
    latest_attitude = None
    phase = "preflight"
    roll_pwm = 1500
    completed = False
    streams_requested = False
    last_request = 0.0
    required_ekf = 1 | 2 | 4 | 16 | 32
    try:
        while running and plane.poll() is None:
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
                elif kind == "GPS_RAW_INT":
                    gps_fix = int(message.fix_type)
                elif kind == "EKF_STATUS_REPORT":
                    ekf_flags = int(message.flags)
                elif kind == "SIMSTATE" and released and latest_attitude:
                    elapsed_s = (
                        int(latest_attitude.time_boot_ms) - release_boot_ms
                    ) / 1000.0
                    if 0.0 <= elapsed_s <= 12.0:
                        simstate.writerow([
                            f"{elapsed_s:.3f}",
                            f"{math.degrees(message.roll):.6f}",
                            f"{math.degrees(message.pitch):.6f}",
                            f"{math.degrees(message.yaw):.6f}",
                            f"{message.xacc:.9f}",
                            f"{message.yacc:.9f}",
                            f"{message.zacc:.9f}",
                            f"{message.xgyro:.9f}",
                            f"{message.ygyro:.9f}",
                            f"{message.zgyro:.9f}",
                        ])
                        simstate_file.flush()
                elif kind == "ATTITUDE":
                    latest_attitude = message
                    if released:
                        elapsed_s = (int(message.time_boot_ms) - release_boot_ms) / 1000.0
                        if elapsed_s >= 12.2:
                            completed = True
                            break
                        next_phase, next_roll_pwm = roll_command(elapsed_s)
                        if (next_phase, next_roll_pwm) != (phase, roll_pwm):
                            phase, roll_pwm = next_phase, next_roll_pwm
                            rc_override(connection, roll_pwm, 1775)
                        telemetry.writerow([
                            f"{elapsed_s:.3f}", phase, roll_pwm, 1500, 1775, 1500,
                            mode, int(armed),
                            f"{math.degrees(message.roll):.6f}",
                            f"{math.degrees(message.pitch):.6f}",
                            f"{math.degrees(message.yaw):.6f}",
                        ])
                        telemetry_file.flush()

            now = time.monotonic()
            if connection.target_system and not streams_requested:
                request_message(
                    connection, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5
                )
                request_message(
                    connection, mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5
                )
                request_message(
                    connection, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20
                )
                request_message(
                    connection, mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE, 20
                )
                streams_requested = True
            if connection.target_system and now - last_request >= 0.5:
                rc_override(connection, roll_pwm, 1775 if armed else 1000)
                if mode != "FBWA":
                    mode_id = connection.mode_mapping().get("FBWA")
                    if mode_id is not None:
                        connection.mav.set_mode_send(
                            connection.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            mode_id,
                        )
                elif gps_fix >= 3 and (ekf_flags & required_ekf) == required_ekf:
                    if not armed:
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
                    elif not released and latest_attitude is not None:
                        release_boot_ms = int(latest_attitude.time_boot_ms)
                        phase, roll_pwm = roll_command(0.0)
                        rc_override(connection, roll_pwm, 1775)
                        release_unreal(
                            host, int(os.environ.get("UVD_CONTROL_PORT", "9003"))
                        )
                        print("ArduPlane is armed in FBWA; releasing vehicle dynamics", flush=True)
                        released = True
                last_request = now
    finally:
        if plane.poll() is None:
            plane.terminate()
            try:
                plane.wait(timeout=5)
            except subprocess.TimeoutExpired:
                plane.kill()
                plane.wait()
        if completed:
            (ardupilot_dir / "eeprom.bin").unlink(missing_ok=True)
            (ardupilot_dir / "logs" / "LASTLOG.TXT").unlink(missing_ok=True)
            shutil.rmtree(ardupilot_dir / "terrain", ignore_errors=True)
        telemetry_file.close()
        simstate_file.close()
    return 0 if completed else (plane.returncode or 1)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    sys.exit(main())
