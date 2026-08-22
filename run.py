#!/usr/bin/env python3
"""Build and run an explicit Unreal + Cesium + ArduPilot demonstration."""

import json
import os
from datetime import datetime
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT / "unreal" / "UnrealVehicleDynamics.uproject"
PROFILES = {
    "plane": {
        "config": ROOT / "examples" / "run.json",
        "image": "uvd-ardupilot:v1",
        "dockerfile": ROOT / "ardupilot" / "Dockerfile",
        "run_prefix": "fbwa-roll",
    },
    "otter": {
        "config": ROOT / "examples" / "otter-run.json",
        "image": "uvd-ardurover:v1",
        "dockerfile": ROOT / "ardupilot" / "Dockerfile.rover",
        "run_prefix": "acro-turn",
    },
    "otter-open-loop": {
        "config": ROOT / "examples" / "otter-run.json",
        "image": None,
        "dockerfile": None,
        "run_prefix": "otter-open-loop",
    },
}


def load_local_env():
    path = ROOT / ".env"
    if not path.is_file():
        return
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        name = name.strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        if name:
            os.environ.setdefault(name, value)


def unreal_editor():
    configured = os.environ.get("UVD_UNREAL_EDITOR")
    candidates = [
        Path(configured) if configured else None,
        Path("/Users/Shared/Epic Games/UE_5.8/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor"),
        Path("/Users/Shared/Epic Games/UE_5.8/Engine/Binaries/Mac/UnrealEditor"),
        Path("/opt/UnrealEngine/Engine/Binaries/Linux/UnrealEditor"),
    ]
    for candidate in candidates:
        if candidate and candidate.is_file():
            return candidate.resolve()
    raise RuntimeError("Unreal Editor 5.8 was not found; set UVD_UNREAL_EDITOR")


def engine_root(editor):
    for parent in editor.parents:
        if (parent / "Build" / "Build.version").is_file():
            return parent
    raise RuntimeError("could not find the Unreal Engine root")


def build(editor, profile):
    subprocess.run(["cmake", "-S", str(ROOT), "-B", str(ROOT / "build")], check=True)
    subprocess.run(["cmake", "--build", str(ROOT / "build")], check=True)
    engine = engine_root(editor)
    if sys.platform == "darwin":
        script = engine / "Build" / "BatchFiles" / "Mac" / "Build.sh"
        platform = "Mac"
    else:
        script = engine / "Build" / "BatchFiles" / "Linux" / "Build.sh"
        platform = "Linux"
    subprocess.run([
        str(script), "UnrealVehicleDynamicsEditor", platform, "Development",
        f"-Project={PROJECT}", "-WaitMutex", "-NoHotReloadFromIDE",
    ], check=True)
    if profile["image"]:
        subprocess.run([
            "docker", "build", "-t", profile["image"], "-f",
            str(profile["dockerfile"]), str(ROOT),
        ], check=True)


def main():
    load_local_env()
    profile_name = sys.argv[1] if len(sys.argv) > 1 else "plane"
    if profile_name not in PROFILES or len(sys.argv) > 2:
        choices = " | ".join(PROFILES)
        raise RuntimeError(f"usage: ./run.py [{choices}]")
    profile = PROFILES[profile_name]
    config_path = profile["config"]
    editor = unreal_editor()
    build(editor, profile)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = ROOT / "runs" / f"{profile['run_prefix']}-{timestamp}"
    if run_dir.exists():
        run_dir = run_dir.with_name(f"{run_dir.name}-{os.getpid()}")
    run_dir.mkdir(parents=True)
    print(f"uvd: writing run to {run_dir}", flush=True)
    config = json.loads(config_path.read_text())
    origin = config["origin"]
    home = ",".join(str(value) for value in [
        origin["latitude_deg"], origin["longitude_deg"],
        origin["altitude_msl_m"], origin["heading_deg"],
    ])
    rate = round(1.0 / config["fixed_dt_s"])
    container_name = f"uvd-{profile_name}-{os.getpid()}"
    unreal_command = [
        str(editor), str(PROJECT), "-game", "-log", "-novsync", "-Multiprocess",
        f"-UvdRun={config_path}", f"-UvdFixedDt={config['fixed_dt_s']}",
        f"-UvdTruth={run_dir / 'truth.csv'}",
    ]
    if profile_name == "otter-open-loop":
        unreal_command.append("-UvdOpenLoop")
    unreal = subprocess.Popen(unreal_command)
    if not profile["image"]:
        try:
            return unreal.wait()
        except KeyboardInterrupt:
            unreal.terminate()
            return unreal.wait()

    docker_command = [
        "docker", "run", "--rm", "--name", container_name,
        "--volume", f"{run_dir}:/run",
    ]
    if sys.platform.startswith("linux"):
        docker_command += ["--add-host", "host.docker.internal:host-gateway"]
    docker_command += [
        "-e", "UVD_HOST_ADDRESS=host.docker.internal",
        "-e", f"UVD_HOME={home}",
        "-e", f"UVD_RATE_HZ={rate}",
        "-e", f"UVD_CONTROL_PORT={config['controller']['control_port']}",
        "-e", "UVD_RUN_DIR=/run",
        profile["image"],
    ]
    controller = subprocess.Popen(docker_command)
    stopping = False

    def stop(_signal=None, _frame=None):
        nonlocal stopping
        if _signal is not None:
            stopping = True
        if unreal.poll() is None:
            unreal.terminate()
        subprocess.run(
            ["docker", "stop", "--timeout", "3", container_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while unreal.poll() is None and controller.poll() is None:
            time.sleep(0.5)
        controller_finished = controller.poll() is not None
    finally:
        stop()
        unreal.wait()
        controller.wait()
    if stopping:
        return 0
    if controller_finished:
        if controller.returncode:
            return controller.returncode
        subprocess.run([
            "docker", "run", "--rm", "--volume", f"{run_dir}:/run",
            "--entrypoint", "/opt/ardupilot-venv/bin/python", profile["image"],
            "/opt/ardupilot/report.py", "/run",
        ], check=True)
        print(f"uvd: report ready at {run_dir / 'report.html'}", flush=True)
        return 0
    return unreal.returncode or controller.returncode


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"uvd: {error}", file=sys.stderr)
        sys.exit(1)
