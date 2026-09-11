"""Explicit, bounded PX4-console P1 trial in a private network/PID namespace.

Imports are runtime-free. No ROS/MAVSDK/MAVROS control publisher is used.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import time
import uuid

from p1_contract import completed_console_response, evaluate_hover, load_config, parse_topic, require_gates


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fresh_observation(read, previous_time, deadline):
    """Retain repeated reads, but count only advancing simulator observations."""
    while time.monotonic() < deadline:
        observation = read()
        if previous_time is None or observation["t"] > previous_time:
            return observation
        if observation["t"] < previous_time:
            raise RuntimeError("Simulation time moved backwards")
        time.sleep(0.02)
    raise TimeoutError("No fresh observation before wall deadline")


class Console:
    def __init__(self, command, cwd, env, log, deadline):
        import pty
        master, slave = pty.openpty()
        self.fd = master
        self.log = log
        self.deadline = deadline
        self.process = subprocess.Popen(command, cwd=cwd, env=env, stdin=slave,
                                        stdout=slave, stderr=slave, start_new_session=True)
        os.close(slave)

    def prompt(self, timeout=8, command=None):
        end = min(time.monotonic() + timeout, self.deadline)
        text = ""
        while time.monotonic() < end:
            if select.select([self.fd], [], [], 0.05)[0]:
                chunk = os.read(self.fd, 65536).decode(errors="replace")
                if not chunk:
                    raise RuntimeError("PX4 console closed")
                self.log.write(chunk)
                self.log.flush()
                text += chunk
                response = completed_console_response(text, command)
                if response is not None:
                    return response
            if self.process.poll() is not None:
                raise RuntimeError("PX4 exited before prompt")
        raise TimeoutError("PX4 prompt/wall deadline exceeded: " + (command or "boot"))

    def command(self, value):
        os.write(self.fd, (value + "\n").encode())
        return self.prompt(command=value)

    def topic(self, name):
        return parse_topic(self.command("listener " + name + " -n 1"))


def trial(args):
    require_gates(os.environ, args.allow_simulated_flight)
    # The wrapper creates a new PID namespace and launches this process as init.
    if os.getpid() != 1 or set(os.listdir("/sys/class/net")) != {"lo"}:
        # /sys may reflect the original namespace; ip is the authoritative probe.
        interfaces = json.loads(subprocess.check_output(["ip", "-j", "link"], text=True))
        if os.getpid() != 1 or {i["ifname"] for i in interfaces} != {"lo"}:
            raise RuntimeError("Private PID/network namespace is required")
    sim = Path(__file__).resolve().parents[1]
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    config = load_config(sim / "configs/p1_smoke.yaml")
    run = root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-p1-" + uuid.uuid4().hex[:8])
    run.mkdir(parents=True)
    work = run / "rootfs"
    work.mkdir()
    temporary = run / "tmp"
    temporary.mkdir()
    px4 = root / "upstream/PX4-Autopilot"
    build = px4 / "build/px4_sitl_default_linux"
    binary = build / "bin/px4"
    if not binary.is_file():
        raise RuntimeError("Missing built PX4")
    current_lock = json.loads((sim / "configs/versions.lock.yaml").read_text())
    built = json.loads((root / "state/p0-built.json").read_text())
    if built["sources"] != current_lock["sources"] or built["platform"] != current_lock["platform"]:
        raise RuntimeError("Build receipt does not match dependency lock")
    if built["measured"]["px4_binary_sha256"] != digest(binary):
        raise RuntimeError("PX4 binary differs from successful build receipt")
    for relative, expected in built["measured"]["asset_hashes"].items():
        if digest(px4 / relative) != expected:
            raise RuntimeError("Model/world differs from successful build receipt")
    for name in ("gz_env.sh",):
        (work / name).write_bytes((build / "rootfs" / name).read_bytes())
    allowed_env = ("HOME", "PATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
                   "DISPLAY", "WAYLAND_DISPLAY", "XDG_RUNTIME_DIR", "GZ_CONFIG_PATH",
                   "GZ_SIM_SYSTEM_PLUGIN_PATH", "GZ_SIM_RESOURCE_PATH", "GZ_GUI_PLUGIN_PATH")
    env = {k: os.environ[k] for k in allowed_env if k in os.environ}
    env["TMPDIR"] = str(temporary)
    env.update({"PX4_SIM_MODEL": "gz_x500", "PX4_GZ_WORLD": "default", "GZ_DISTRO": "harmonic",
                "GZ_PARTITION": run.name, "GZ_IP": "127.0.0.1", "ROS_DOMAIN_ID": "71",
                "PX4_PARAM_MIS_TAKEOFF_ALT": str(config["target_height_m"]),
                "PX4_PARAM_SDLOG_MODE": "1"})
    if args.headless:
        env["HEADLESS"] = "1"
    command = [str(binary), "-i", "71", "-w", str(work), str(build / "etc")]
    summary = {"boot": "not_run", "flight_smoke": "not_run", "hover": None,
               "landed_disarmed": None, "mode": "headless_physics" if args.headless else "gui_requested",
               "run_id": run.name, "control_owner": "local_px4_console", "launch": command,
               "config": config, "config_sha256": digest(sim / "configs/p1_smoke.yaml"),
               "lock_sha256": digest(sim / "configs/versions.lock.yaml"), "binary_sha256": digest(binary),
               "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=sim, text=True).strip(),
               "isolation": "private user/mount/PID/network namespaces; loopback only; unique GZ_PARTITION",
               "run_environment": {k: env[k] for k in env if k.startswith(("PX4_", "GZ_", "ROS_", "HEADLESS"))},
               "cleanup": "pending", "failure": None}
    summary["operator_script_sha256"] = {p.name: digest(p) for p in sorted((sim / "scripts").glob("*")) if p.is_file()}
    for name in ("versions.lock.yaml", "p1_smoke.yaml"):
        (run / name).write_bytes((sim / "configs" / name).read_bytes())
    start_wall = time.monotonic()
    console = None
    qgc = None
    qgc_log = None
    samples = []
    hover = []
    flight_started = False
    print("P1 evidence: " + str(run), flush=True)

    def save():
        (run / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    def read_observation():
        pos = console.topic("vehicle_local_position")
        status = console.topic("vehicle_status")
        landed = console.topic("vehicle_land_detected")
        if any(key not in pos for key in ("timestamp", "x", "y", "z", "xy_valid", "z_valid")):
            raise RuntimeError("Missing local-position measurements")
        s = {"t": pos["timestamp"] / 1e6, "x": pos["x"], "y": pos["y"], "z": pos["z"],
             "position": pos, "status": status, "land": landed}
        if not all(isinstance(s[k], (int, float)) and math.isfinite(s[k]) for k in ("t", "x", "y", "z")):
            raise RuntimeError("Non-finite position")
        if samples and s["t"] - samples[0]["t"] > config["simulation_deadline_s"]:
            raise TimeoutError("Simulation deadline exceeded")
        if time.monotonic() - start_wall > config["wall_deadline_s"]:
            raise TimeoutError("Wall deadline exceeded")
        with (run / "observations.jsonl").open("a") as out:
            out.write(json.dumps(s) + "\n")
        return s

    def sample():
        s = fresh_observation(read_observation, samples[-1]["t"] if samples else None,
                              start_wall + config["wall_deadline_s"])
        samples.append(s)
        return s

    def healthy(s):
        return (s["position"]["xy_valid"] is True and s["position"]["z_valid"] is True
                and s["status"].get("pre_flight_checks_pass") is True
                and s["status"].get("failsafe") is False)

    try:
        save()
        if args.qgc_monitor:
            qgc_dir = root / "qgc-v5.1.4"
            appimage = qgc_dir / "QGroundControl-x86_64.AppImage"
            if digest(appimage) != "1c4ac089abfaac6c6fcd75c7b477ea18da1bc3592cddca5ab1a19c1a13410e65":
                raise RuntimeError("QGroundControl artifact does not match official release digest")
            qgc_config = run / "qgc-config/QGroundControl/QGroundControl.ini"
            qgc_config.parent.mkdir(parents=True)
            qgc_config.write_text((sim / "configs/qgc-monitor.ini").read_text().replace("@RUN_DATA@", str(run / "qgc-data")))
            qgc_env = {k: env[k] for k in ("HOME", "PATH", "TMPDIR", "DISPLAY", "WAYLAND_DISPLAY", "XDG_RUNTIME_DIR") if k in env}
            qgc_env.update({"XDG_CONFIG_HOME": str(run / "qgc-config"),
                            "XDG_CACHE_HOME": str(run / "qgc-cache"), "XDG_DATA_HOME": str(run / "qgc-data")})
            if args.headless:
                qgc_env.update({"QT_QPA_PLATFORM": "offscreen", "QT_QUICK_BACKEND": "software"})
            qgc_log = (run / "qgc-monitor.log").open("w")
            qgc = subprocess.Popen([str(qgc_dir / "squashfs-root/AppRun")], env=qgc_env,
                                   stdin=subprocess.DEVNULL, stdout=qgc_log, stderr=qgc_log, start_new_session=True)
            summary["qgc_monitor"] = {"namespace_pid": qgc.pid, "version": "5.1.4",
                                       "artifact_sha256": digest(appimage), "config_sha256": digest(qgc_config),
                                       "role": "telemetry monitoring only; no UI control interaction"}
        with (run / "px4-console.log").open("w") as log:
            console = Console(command, work, env, log, start_wall + config["wall_deadline_s"])
            summary["px4_namespace_pid"] = console.process.pid
            boot_text = console.prompt(90)
            if "Gazebo world is ready" not in boot_text or "x500_71" not in boot_text:
                raise RuntimeError("Expected world/model bring-up evidence missing")
            (run / "processes.txt").write_text(subprocess.check_output(["ps", "-eo", "pid,ppid,user,comm,args"], text=True))
            (run / "endpoints.txt").write_text(subprocess.check_output(["ss", "-lunp"], text=True))
            (run / "gazebo-topics.txt").write_text(subprocess.check_output(["gz", "topic", "-l"], env=env, text=True, timeout=10))
            console.command("param show -a")
            console.command("commander check")
            logging = console.command("logger status")
            if "file" not in logging.lower():
                raise RuntimeError("Logger status lacks file evidence")
            for _ in range(150):
                if qgc is not None and qgc.poll() is not None:
                    raise RuntimeError("QGroundControl monitor exited; inspect qgc-monitor.log")
                s = sample()
                if healthy(s) and len(samples) >= 2:
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError("Normal preflight/estimator health did not become valid")
            if not list(work.rglob("*.ulg")):
                raise RuntimeError("No active ULog file")
            if s["land"].get("landed") is not True or s["status"].get("arming_state") != 1:
                raise RuntimeError("Initial landed/disarmed state not verified")
            summary["boot"] = "passed"
            ground_z = s["z"]
            summary["initial_ground_local_z"] = ground_z
            if args.allow_simulated_flight:
                summary["flight_smoke"] = "failed"
                flight_started = True
                console.command("commander takeoff")  # Normal checks; no force-arm.
                while True:
                    s = sample()
                    if not healthy(s):
                        raise RuntimeError("Flight health/estimation lost")
                    if (s["status"].get("arming_state") == 2 and s["land"].get("landed") is False
                            and s["status"].get("nav_state") == 4 and abs(s["position"].get("vz", math.inf)) <= 0.1):
                        if abs(ground_z - s["z"] - config["target_height_m"]) <= config["height_tolerance_m"]:
                            break
                    time.sleep(0.1)
                hover.append(s)
                while hover[-1]["t"] - hover[0]["t"] < config["hover_duration_sim_s"]:
                    time.sleep(0.1)
                    s = sample()
                    if not healthy(s) or s["status"].get("arming_state") != 2 or s["land"].get("landed") is not False:
                        raise RuntimeError("Hover flight state invalid")
                    hover.append(s)
                summary["hover"] = evaluate_hover(hover, config, ground_z)
                if summary["hover"]["status"] != "passed":
                    raise RuntimeError("Fixed hover acceptance rule failed")
                console.command("commander land")
                while True:
                    s = sample()
                    if s["land"].get("landed") is True and s["status"].get("arming_state") == 1:
                        summary["landed_disarmed"] = True
                        break
                    if s["status"].get("failsafe") is not False:
                        raise RuntimeError("Failsafe during landing")
                    time.sleep(0.1)
                summary["flight_smoke"] = "passed"
            else:
                summary["landed_disarmed"] = True
            console.command("logger stop")
            os.write(console.fd, b"shutdown\n")
            console.process.wait(timeout=15)
    except (Exception, KeyboardInterrupt) as exc:
        summary["failure"] = str(exc)
        if summary["boot"] != "passed":
            summary["boot"] = "failed"
        if args.allow_simulated_flight and not flight_started:
            summary["flight_smoke"] = "blocked"
        print("P1 stopped: " + str(exc), flush=True)
    finally:
        # Signal only the process group created here. No further vehicle commands
        # on failure. Namespace init exit terminates any remaining descendants.
        if console is not None:
            try:
                os.killpg(console.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                console.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(console.process.pid, signal.SIGKILL)
                console.process.wait(timeout=5)
            os.close(console.fd)
        if qgc is not None:
            try:
                os.killpg(qgc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                qgc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(qgc.pid, signal.SIGKILL)
                qgc.wait(timeout=5)
        if qgc_log is not None:
            qgc_log.close()
        summary["ulog_files"] = [{"path": str(p), "bytes": p.stat().st_size, "sha256": digest(p)} for p in work.rglob("*.ulg")]
        if not summary["ulog_files"] and summary["flight_smoke"] == "passed":
            summary["flight_smoke"] = "failed"
            summary["failure"] = "Missing ULog evidence"
        summary["cleanup"] = "owned PX4 group stopped; remaining namespace descendants terminate at init exit"
        summary["wall_duration_s"] = time.monotonic() - start_wall
        summary["observation_count"] = len(samples)
        save()
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary["boot"] == "passed" and (not args.allow_simulated_flight or summary["flight_smoke"] == "passed") else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true", required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--qgc-monitor", action="store_true")
    raise SystemExit(trial(parser.parse_args()))
