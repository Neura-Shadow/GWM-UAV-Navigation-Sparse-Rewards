"""P2-owned namespace supervisor. Console performs inspection, never flight."""
import argparse
import json
import os
from pathlib import Path
import select
import re
import signal
import subprocess
import time
import uuid

from p1_contract import require_gates
from p1_runner import Console, digest
from p2_build import package_hash, package_manifest


def trial(args):
    require_gates(os.environ, args.allow_simulated_flight)
    interfaces = json.loads(subprocess.check_output(["ip", "-j", "link"], text=True))
    if os.getpid() != 1 or {item["ifname"] for item in interfaces} != {"lo"}:
        raise ValueError("Owned private PID/network namespace required")
    sim = Path(__file__).resolve().parents[1]
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home()/"uav_autonomy")))
    config = json.loads((sim/"configs/p2_control.yaml").read_text())
    receipt = json.loads((root/"state/p2-built.json").read_text())
    if package_hash(package_manifest(sim)) != receipt["package_hash"]:
        raise ValueError("Controller sources differ from tested build mirror")
    source = Path(receipt["workspace"])/"src/gwm_px4_control"
    for name, expected in receipt["source_files"].items():
        if digest(source/name) != expected:
            raise ValueError("Linux mirror changed: "+name)
    px4 = root/"upstream/PX4-Autopilot"
    build = px4/"build/px4_sitl_default_linux"
    binary = build/"bin/px4"
    baseline = json.loads((root/"state/p0-built.json").read_text())
    lock = json.loads((sim/"configs/versions.lock.yaml").read_text())
    if baseline["sources"] != lock["sources"] or baseline["measured"]["px4_binary_sha256"] != digest(binary):
        raise ValueError("P0 binary/source receipt mismatch")
    for name, expected in baseline["measured"]["asset_hashes"].items():
        if digest(px4/name) != expected:
            raise ValueError("Pinned model/world changed: "+name)
    identity = {"package_hash": receipt["package_hash"], "config_sha256": digest(sim/"configs/p2_control.yaml"),
                "clock_bridge_sha256": digest(sim/"configs/p2_clock_bridge.yaml"),
                "qgc_profile_sha256": digest(sim/"configs/qgc-monitor.ini"),
                "lock_sha256": digest(sim/"configs/versions.lock.yaml"), "binary_sha256": digest(binary),
                "topic_contract": receipt["topic_contract"],
                "launcher_sha256": {name: digest(sim/"scripts"/name) for name in
                    ("p2_runner.py", "run_p2_control.sh", "p2_build.py", "common.sh", "p1_runner.py", "p1_contract.py")}}
    if args.allow_simulated_flight:
        observation = json.loads((root/"state/p2-connection.json").read_text())
        if observation["identity"] != identity or observation["status"] != "passed":
            raise ValueError("Read-only connection stage must pass for these exact inputs")
    kind = "flight" if args.allow_simulated_flight else "observe"
    run = root/"runs"/(time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())+"-p2-"+kind+"-"+uuid.uuid4().hex[:8])
    run.mkdir(parents=True)
    for folder in ("rootfs", "tmp", "qgc-config/QGroundControl"):
        (run/folder).mkdir(parents=True)
    work = run/"rootfs"
    (work/"gz_env.sh").write_bytes((build/"rootfs/gz_env.sh").read_bytes())
    for name in ("p2_control.yaml", "p2_clock_bridge.yaml", "versions.lock.yaml"):
        (run/name).write_bytes((sim/"configs"/name).read_bytes())
    (run/"p2-build-receipt.json").write_text(json.dumps(receipt, indent=2, allow_nan=False)+"\n")
    (run/"qgc-config/QGroundControl/QGroundControl.ini").write_text(
        (sim/"configs/qgc-monitor.ini").read_text().replace("@RUN_DATA@", str(run/"qgc-data")))
    allowed = ("HOME", "PATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH",
               "DISPLAY", "WAYLAND_DISPLAY", "XDG_RUNTIME_DIR", "GZ_CONFIG_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH",
               "GZ_SIM_RESOURCE_PATH", "GZ_GUI_PLUGIN_PATH", "PYTHONNOUSERSITE", "ROS_DISTRO", "ROS_VERSION",
               "GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS",
               "GWM_ALLOW_PX4_LAUNCH", "GWM_ALLOW_SITL_COMMANDS")
    env = {key: os.environ[key] for key in allowed if key in os.environ}
    env.update({"TMPDIR": str(run/"tmp"), "ROS_DOMAIN_ID": str(config["dds_domain"]),
                "GZ_PARTITION": run.name, "GZ_IP": "127.0.0.1", "GZ_DISTRO": "harmonic",
                "PX4_SIM_MODEL": "gz_x500", "PX4_GZ_WORLD": "default", "PX4_UXRCE_DDS_NS": "px4_71",
                "PX4_UXRCE_DDS_PORT": "8888", "PX4_PARAM_UXRCE_DDS_SYNCT": "0",
                "PX4_PARAM_SDLOG_MODE": "1", "GWM_P2_OWNED_NAMESPACE": "1",
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"})
    if args.headless:
        env["HEADLESS"] = "1"
    start = time.monotonic()
    summary = {"run_id": run.name, "kind": kind, "identity": identity, "config": config,
               "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=sim, text=True).strip(),
               "status": "incomplete", "failure": None, "controller_result": None,
               "mode": "headless_physics" if args.headless else "gui_requested",
               "isolation": "private user/mount/PID/network namespace, loopback only, exclusive P1/P2 lock",
               "qgc_monitor_present": True, "manual_flight_commands": False,
               "control_owner": "gwm_px4_control" if args.allow_simulated_flight else "none_read_only",
               "clock": {"gz_topic": "/world/default/clock", "ros_topic": "/clock", "direction": "GZ_TO_ROS",
                         "use_sim_time": True, "UXRCE_DDS_SYNCT": 0}, "processes": []}
    processes, streams = [], []
    console = None
    print("P2 evidence: "+str(run), flush=True)

    def save():
        (run/"summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")

    def launch(name, command, process_env=env):
        stream = (run/(name+".log")).open("w")
        streams.append(stream)
        process = subprocess.Popen(command, env=process_env, cwd=work, stdout=stream, stderr=stream,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
        processes.append((name, process))
        summary["processes"].append({"name": name, "pid": process.pid, "command": command})
        return process

    try:
        save()
        qgc_root = root/"qgc-v5.1.4"
        if digest(qgc_root/"QGroundControl-x86_64.AppImage") != lock["qgc_monitor"]["sha256"]:
            raise ValueError("QGC artifact mismatch")
        qenv = {k: env[k] for k in ("HOME", "PATH", "TMPDIR", "DISPLAY", "WAYLAND_DISPLAY", "XDG_RUNTIME_DIR") if k in env}
        qenv.update({"XDG_CONFIG_HOME": str(run/"qgc-config"), "XDG_DATA_HOME": str(run/"qgc-data"),
                     "XDG_CACHE_HOME": str(run/"qgc-cache")})
        if args.headless:
            qenv.update({"QT_QPA_PLATFORM": "offscreen", "QT_QUICK_BACKEND": "software"})
        launch("qgc-monitor", [str(qgc_root/"squashfs-root/AppRun")], qenv)
        denv = dict(env)
        denv["LD_LIBRARY_PATH"] = str(root/"dds-install/lib")+":"+env.get("LD_LIBRARY_PATH", "")
        launch("dds-agent", [str(root/"dds-install/bin/MicroXRCEAgent"), "udp4", "-p", "8888", "-v", "4"], denv)
        px4log = (run/"px4-console.log").open("w")
        streams.append(px4log)
        command = [str(binary), "-i", "71", "-w", str(work), str(build/"etc")]
        console = Console(command, work, env, px4log, start+config["wall_deadline_s"]+90)
        summary["processes"].append({"name": "px4", "pid": console.process.pid, "command": command})
        boot = console.prompt(90)
        if "Gazebo world is ready" not in boot or "x500_71" not in boot:
            raise ValueError("Expected owned world/model not observed")
        parameters = console.command("param show -a")
        if config.get("reference_policy") == "p2-estimator-reference-v2":
            clean_parameters = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", parameters)
            required = {"EKF2_MAG_TYPE": 0, "SENS_IMU_MODE": 1, "SENS_MAG_MODE": 1,
                        "EKF2_MULTI_IMU": 0, "EKF2_MULTI_MAG": 0}
            effective = {}
            for name, expected in required.items():
                found = re.search(r"\b"+name+r"\s+\[[-0-9,]+\]\s*:\s*([-0-9.]+)", clean_parameters)
                if found is None or float(found.group(1)) != expected:
                    raise ValueError("Unsupported estimator launch contract: "+name)
                effective[name] = float(found.group(1))
            summary["estimator_launch_contract"] = effective
            (run/"ekf2-status.txt").write_text(console.command("ekf2 status"))
        sync = console.command("param show UXRCE_DDS_SYNCT")
        if "UXRCE_DDS_SYNCT" not in sync or ": 0" not in sync:
            raise ValueError("Effective UXRCE_DDS_SYNCT=false not verified")
        console.command("uxrce_dds_client status")
        console.command("logger status")
        topics = subprocess.check_output(["gz", "topic", "-l"], env=env, text=True, timeout=10)
        (run/"gazebo-topics.txt").write_text(topics)
        if "/world/default/clock" not in topics.splitlines():
            raise ValueError("Configured Gazebo clock topic absent")
        launch("clock-bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                               "-p", "config_file:="+str(run/"p2_clock_bridge.yaml"), "-p", "use_sim_time:=true"])
        controller_command = [str(Path(receipt["install"])/"gwm_px4_control/lib/gwm_px4_control/p2_control"),
                              "--config", str(run/"p2_control.yaml"), "--run-dir", str(run)]
        if args.allow_simulated_flight:
            controller_command.append("--flight")
        controller = launch("ros-controller", controller_command)
        (run/"processes.txt").write_text(subprocess.check_output(["ps", "-eo", "pid,ppid,user,comm,args"], text=True))
        (run/"endpoints.txt").write_text(subprocess.check_output(["ss", "-lunp"], text=True))
        save()
        while controller.poll() is None:
            if time.monotonic()-start > config["wall_deadline_s"]+90:
                raise TimeoutError("supervisor_hard_wall_deadline")
            for name, process in processes:
                if process is not controller and process.poll() is not None:
                    raise RuntimeError("owned_process_exited:"+name)
            if console.process.poll() is not None:
                raise RuntimeError("px4_process_exited")
            if select.select([console.fd], [], [], 0.1)[0]:
                px4log.write(os.read(console.fd, 65536).decode(errors="replace"))
                px4log.flush()
        summary["controller_exit_code"] = controller.returncode
        result_path = run/"controller-result.json"
        if result_path.exists():
            summary["controller_result"] = json.loads(result_path.read_text())
        # Read-only final inspection is independent of ROS command acceptance.
        status, landed = console.topic("vehicle_status"), console.topic("vehicle_land_detected")
        summary["final_console_state"] = {"status": status, "landed": landed}
        if controller.returncode != 0:
            raise RuntimeError("controller_failed; inspect controller result and log")
        if status.get("arming_state") != 1 or landed.get("landed") is not True:
            raise ValueError("Final landed/disarmed state not independently observed")
        console.command("logger stop")
        os.write(console.fd, b"shutdown\n")
        console.process.wait(timeout=15)
        summary["status"] = "passed"
    except (Exception, KeyboardInterrupt) as exc:
        summary["status"], summary["failure"] = "failed", str(exc) or "interrupted"
        print("P2 stopped: "+summary["failure"], flush=True)
    finally:
        owned = processes + ([("px4", console.process)] if console else [])
        for name, process in reversed(owned):
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            for record in summary["processes"]:
                if record["pid"] == process.pid:
                    record["exit_code"] = process.poll()
        if console:
            os.close(console.fd)
        for stream in streams:
            stream.close()
        summary["ulog_files"] = [{"path": str(p), "bytes": p.stat().st_size, "sha256": digest(p)} for p in work.rglob("*.ulg")]
        summary["rosbag_files"] = [{"path": str(p), "bytes": p.stat().st_size, "sha256": digest(p)} for p in (run/"rosbag").glob("*") if p.is_file()]
        if summary["status"] == "passed" and (not summary["ulog_files"] or not any(p["path"].endswith("metadata.yaml") for p in summary["rosbag_files"])):
            summary["status"], summary["failure"] = "failed", "Required ULog/rosbag metadata missing"
        summary["cleanup"] = "owned groups stopped; namespace init exit terminates remaining descendants"
        summary["wall_duration_s"] = time.monotonic()-start
        save()
    if summary["status"] == "passed" and args.observe:
        (root/"state/p2-connection.json").write_text(json.dumps({"status": "passed", "identity": identity,
                                                                 "run_id": run.name}, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"run_id": run.name, "status": summary["status"], "failure": summary["failure"]}, indent=2), flush=True)
    return 0 if summary["status"] == "passed" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true", required=True)
    parser.add_argument("--observe", action="store_true")
    parser.add_argument("--allow-simulated-flight", action="store_true")
    parser.add_argument("--headless", action="store_true")
    raise SystemExit(trial(parser.parse_args()))
