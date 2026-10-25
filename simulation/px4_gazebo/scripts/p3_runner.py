"""P3 owned-namespace supervisor: one runtime scope per invocation.

Scopes
  readiness  PX4 (explicitly selected MC causal-trace build) + Gazebo x500_depth,
             DDS Agent, unattended QGC monitor, clock/sensor bridges, depth adapter,
             evaluator-only source probe and the unchanged P2 controller in its
             read-only connection mode. Vehicle stays landed/disarmed.
  fixture    Gazebo server + one versioned ground world, sensor bridge, clock bridge,
             depth adapter and source probe. No autopilot, no flight API at all.
  flight     Readiness stack plus the unchanged P2 controller with --flight
             (the accepted P2 mission). Requires the flight gates.

There is no automatic retry, re-arm, force-arm/disarm, preflight bypass or
second flight owner. Shutdown is ordered and every exit is recorded raw; the
independent evaluator classifies graceful cleanup. Finalization always runs.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import select
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid

SIM = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SIM / "scripts"), str(SIM / "validation"), str(SIM / "ros2_ws/src/gwm_px4_control")]
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402
import p3_manifest  # noqa: E402

GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
PARAMETER_BRIDGE = "/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"
SOFTWARE_RENDERERS = re.compile(r"llvmpipe|softpipe|swrast|software rasterizer", re.I)


class Abort(RuntimeError):
    pass


# Owned process bookkeeping -------------------------------------------------------------------
class Owned:
    """One directly launched (or adopted orphan) process; raw facts only."""

    def __init__(self, name, pid, command, stop_method, grace_s, popen=None):
        self.name, self.pid, self.popen = name, pid, popen
        self.record = {"name": name, "pid": pid, "command": command, "stop_method": stop_method, "grace_s": grace_s,
                       "graceful_codes": [0], "launched_monotonic_s": time.monotonic(),
                       "stop_requested_monotonic_s": None, "stop_signal": None, "exit_observed_monotonic_s": None,
                       "exit_code": None, "reaped": False, "forced": False, "remaining_group_members": None}

    def poll(self):
        if self.record["reaped"]:
            return self.record["exit_code"]
        if self.popen is not None:
            code = self.popen.poll()
        else:
            try:
                pid, status = os.waitpid(self.pid, os.WNOHANG)
            except ChildProcessError:  # not yet reparented to PID 1
                return None
            if pid == 0:
                return None
            code = os.waitstatus_to_exitcode(status)
        if code is not None:
            self.record.update(exit_code=code, reaped=True, exit_observed_monotonic_s=time.monotonic(),
                               remaining_group_members=group_members(self.pid))
        return code

    def request_stop(self, sig):
        if self.poll() is not None:
            return False
        self.record["stop_requested_monotonic_s"] = time.monotonic()
        self.record["stop_signal"] = signal.Signals(sig).name
        os.kill(self.pid, sig)
        return True

    def mark_console_stop(self):
        self.record["stop_requested_monotonic_s"] = time.monotonic()
        self.record["stop_signal"] = "console_shutdown"

    def wait(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.poll() is not None:
                return True
            time.sleep(0.02)
        return self.poll() is not None

    def force(self):
        """Escalation is recorded as a failed cleanup, never as graceful."""
        if self.poll() is None:
            self.record["forced"] = True
            try:
                os.killpg(self.pid, signal.SIGKILL)
            except ProcessLookupError:
                os.kill(self.pid, signal.SIGKILL)
            self.wait(5)
        members = group_members(self.pid)
        for pid in members:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if members:
            self.record["remaining_group_members"] = members


def group_members(pgid):
    members = []
    for entry in Path("/proc").iterdir():
        if entry.name.isdigit() and int(entry.name) != os.getpid():
            try:
                fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            except (OSError, IndexError):
                continue
            if int(fields[2]) == pgid and fields[0] != "Z":
                members.append(int(entry.name))
    return members


def namespace_processes():
    rows = []
    for entry in Path("/proc").iterdir():
        if entry.name.isdigit() and int(entry.name) != os.getpid():
            try:
                rows.append({"pid": int(entry.name),
                             "cmdline": (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip(),
                             "state": (entry / "stat").read_text().rsplit(")", 1)[1].split()[0]})
            except OSError:
                continue
    return rows


class ResourceSampler(threading.Thread):
    def __init__(self, run, owned, root):
        super().__init__(name="p3-resources", daemon=True)
        self.path, self.owned, self.root = run / "resources.jsonl", owned, root
        self.stop_event = threading.Event()
        self.peaks = {}

    def run(self):
        ticks = os.sysconf("SC_CLK_TCK")
        with self.path.open("x", buffering=1) as stream:
            while not self.stop_event.wait(1.0):
                row = {"monotonic_s": time.monotonic(), "disk_free_bytes": shutil.disk_usage(self.root).free,
                       "loadavg": os.getloadavg(), "processes": {}}
                meminfo = dict(line.split(":", 1) for line in Path("/proc/meminfo").read_text().splitlines())
                row["mem_available_kb"] = int(meminfo["MemAvailable"].split()[0])
                for item in list(self.owned):
                    try:
                        stat = Path(f"/proc/{item.pid}/stat").read_text().rsplit(")", 1)[1].split()
                        status = dict(line.split(":", 1) for line in Path(f"/proc/{item.pid}/status").read_text().splitlines())
                    except (OSError, ValueError):
                        continue
                    cpu = (int(stat[11]) + int(stat[12])) / ticks
                    rss = int(status.get("VmRSS", "0 kB").split()[0])
                    row["processes"][item.name] = {"cpu_s": cpu, "rss_kb": rss}
                    peak = self.peaks.setdefault(item.name, {"cpu_s": 0, "rss_kb": 0})
                    peak["cpu_s"], peak["rss_kb"] = max(peak["cpu_s"], cpu), max(peak["rss_kb"], rss)
                stream.write(json.dumps(row) + "\n")


class Tail:
    """Incremental JSONL reader for files written by owned processes."""

    def __init__(self, path):
        self.path, self.offset, self.pending, self.last = Path(path), 0, "", None

    def read(self):
        if not self.path.exists():
            return []
        with self.path.open() as stream:
            stream.seek(self.offset)
            chunk = stream.read()
            self.offset = stream.tell()
        lines = (self.pending + chunk).split("\n")
        self.pending = lines.pop()
        rows = [json.loads(line) for line in lines if line]
        if rows:
            self.last = rows[-1]
        return rows


# Supervisor ----------------------------------------------------------------------------------
class Runner:
    def __init__(self, args):
        self.args = args
        self.scope = args.scope
        self.root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
        self.validation = strict_json(SIM / "configs/p3_validation.yaml")
        self.sensors = strict_json(SIM / "configs/p3_sensors.yaml")
        self.owned = []
        self.streams = []
        self.console = None
        self.capture = None
        self.shutting_down = False
        self.sampler = None
        self.start = time.monotonic()
        self.summary = None
        self.run = None
        self.px4_log = None

    # ---- preconditions -------------------------------------------------------------------
    def preconditions(self):
        missing = [key for key in GATES if os.environ.get(key) != "1"]
        flight = self.scope == "flight"
        if flight and (os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1" or not self.args.allow_simulated_flight):
            missing.append("GWM_ALLOW_SITL_COMMANDS/--allow-simulated-flight")
        if not flight and self.args.allow_simulated_flight:
            raise ValueError("Flight authorization is only valid for the flight scope")
        if missing:
            raise ValueError("Missing explicit gates: " + ", ".join(missing))
        if os.getpid() != 1:
            raise ValueError("Owned private PID/network namespace required")
        interfaces = json.loads(subprocess.check_output(["ip", "-j", "link"], text=True))
        if {item["ifname"] for item in interfaces} != {"lo"}:
            raise ValueError("Owned private PID/network namespace required")
        lock = Path("/proc/1/fd/9").resolve()
        probe = os.open(lock, os.O_RDWR)
        try:
            fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
            raise ValueError("Exclusive P1/P2/P3 workspace lock is not held by this owner")
        except BlockingIOError:
            pass
        finally:
            os.close(probe)
        if shutil.disk_usage(self.root).free < self.validation["disk_reserve_bytes"]:
            raise ValueError("Disk reserve below the required 20 GiB")
        frozen, sha = p3_manifest.verify_current(self.root, self.args.manifest)
        authorization = p3_manifest.require_authorization(self.args.authorization, self.args.gate, sha)
        marker = self.root / "state" / f"p3-gate-{self.args.gate}-{sha}.consumed.json"
        if not marker.exists() or strict_json(marker)["authorization_sha256"] != digest(self.args.authorization):
            raise PermissionError("Gate authorization was not consumed by the gate sequencer")
        if self.scope == "flight" and self.args.gate not in ("B", "C"):
            raise PermissionError("Flights belong to Gate B or Gate C only")
        if self.scope != "flight" and self.args.gate != "A":
            raise PermissionError("Ground scopes belong to Gate A only")
        return frozen, sha, authorization

    # ---- launching -----------------------------------------------------------------------
    def environment(self, extra=None):
        allowed = ("HOME", "PATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH",
                   "DISPLAY", "WAYLAND_DISPLAY", "XDG_RUNTIME_DIR", "GZ_CONFIG_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH",
                   "GZ_SIM_RESOURCE_PATH", "GZ_GUI_PLUGIN_PATH", "PYTHONNOUSERSITE", "ROS_DISTRO", "ROS_VERSION",
                   "GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
        env = {key: os.environ[key] for key in allowed if key in os.environ}
        env.update({"TMPDIR": str(self.run / "tmp"), "ROS_DOMAIN_ID": "71", "GZ_PARTITION": self.run.name,
                    "GZ_IP": "127.0.0.1", "GZ_DISTRO": "harmonic", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                    "GWM_P3_OWNED_NAMESPACE": "1",
                    # WSLg hardware rendering: Mesa picks llvmpipe for headless EGL unless the
                    # D3D12 driver and the discrete adapter are named (verified with an EGL probe).
                    "GALLIUM_DRIVER": "d3d12", "MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA"})
        env.update(extra or {})
        return env

    def launch(self, name, command, env, stop_method="SIGINT", grace=None, cwd=None):
        stream = (self.run / (name + ".log")).open("x")
        self.streams.append(stream)
        process = subprocess.Popen(command, env=env, cwd=cwd or self.run, stdout=stream, stderr=stream,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
        owned = Owned(name, process.pid, command, stop_method,
                      grace if grace is not None else self.validation["graceful_stop_wall_s"], process)
        self.owned.append(owned)
        self.summary["processes"].append(owned.record)
        self.save()
        return owned

    def adopt(self, name, predicate):
        """Adopt the deepest matching descendant (the server itself, not a shell wrapper)."""
        rows = [row for row in namespace_processes() if row["state"] != "Z" and predicate(row["cmdline"])]
        parents = {int(Path(f"/proc/{row['pid']}/stat").read_text().rsplit(")", 1)[1].split()[1]) for row in rows}
        leaves = [row for row in rows if row["pid"] not in parents]
        if len(leaves) != 1:
            raise Abort("expected_single_process:" + name + ":" + str(len(leaves)))
        row = leaves[0]
        owned = Owned(name, row["pid"], row["cmdline"], "SIGINT", self.validation["graceful_stop_wall_s"])
        self.owned.append(owned)
        self.summary["processes"].append(owned.record)
        return owned

    def save(self):
        atomic_json(self.run / "summary.json", self.summary)

    def milestone(self, name, **extra):
        self.summary["startup"]["milestones"].append({"name": name, "monotonic_s": time.monotonic(),
                                                      "wall_since_origin_s": time.monotonic() - self.start, **extra})
        self.save()

    def pump(self):
        """Drain PX4's console, and fail fast on any owned process exit not requested."""
        if self.console is not None:
            while select.select([self.console.fd], [], [], 0)[0]:
                chunk = os.read(self.console.fd, 65536)
                if not chunk:
                    break
                self.px4_log.write(chunk.decode(errors="replace"))
            self.px4_log.flush()
            if self.console.process.poll() is not None:
                raise Abort("px4_process_exited")
        if self.shutting_down:
            return  # the unexpected exit is already the recorded failure; never abort the ordered stop
        for item in self.owned:
            if item.poll() is not None and item.record["stop_requested_monotonic_s"] is None \
                    and item.record["stop_method"] != "completes_own_work":
                raise Abort("owned_process_exited:" + item.name)

    def wait_for(self, predicate, deadline_wall, label):
        while True:
            self.pump()
            value = predicate()
            if value:
                return value
            if time.monotonic() > deadline_wall:
                raise Abort("deadline:" + label)
            time.sleep(0.05)

    def sim_now(self):
        self.clock_tail.read()
        return self.clock_tail.last["sim_s"] if self.clock_tail.last else None

    # ---- shared sensor stack ---------------------------------------------------------------
    def sensor_stack(self, world, require_state, rgb, watch):
        receipt = strict_json(self.root / "state/p3-built.json")
        env = self.environment()
        sensor_env = self.environment({"FASTRTPS_DEFAULT_PROFILES_FILE": str(self.run / "p3_fastdds.xml")})
        # -s -P: no user site or cwd on sys.path, but keep the sourced ROS PYTHONPATH that
        # provides the vendored gz.transport13/gz.msgs10 bindings (-I would drop it).
        self.launch("source-probe", ["/usr/bin/python3", "-s", "-P", str(SIM / "validation/p3_source_probe.py"),
                                     str(self.run), world, self.sensors["model"]] + (["--rgb"] if rgb else [])
                    + (["--watch"] + watch if watch else []), env)
        self.clock_tail = Tail(self.run / "source/clock.jsonl")
        if self.scope == "fixture":
            clock = (SIM / "configs/p2_clock_bridge.yaml").read_text().replace("/world/default/clock",
                                                                               f"/world/{world}/clock")
            (self.run / "clock-bridge.yaml").write_text(clock)
            self.launch("clock-bridge", [PARAMETER_BRIDGE, "--ros-args", "-p",
                                         "config_file:=" + str(self.run / "clock-bridge.yaml"), "-p", "use_sim_time:=true"], env)
        self.bridge = self.launch("sensor-bridge", [PARAMETER_BRIDGE, "--ros-args", "-p",
                                                    "config_file:=" + str(self.run / "p3_sensor_bridge.yaml")], sensor_env)
        adapter = Path(receipt["install"]) / "gwm_sensor_adapter/lib/gwm_sensor_adapter/p3_depth_adapter"
        self.launch("sensor-adapter", [str(adapter), "--config", str(self.run / "p3_sensors.yaml"),
                                       "--run-dir", str(self.run)] + (["--require-px4-state"] if require_state else []),
                    sensor_env)
        self.health_tail = Tail(self.run / "sensors/health.jsonl")
        deadline = self.start + self.validation["startup_wall_s"]
        self.wait_for(lambda: any(r["status"] == "fresh" for r in self.health_tail.read()) or (
            self.health_tail.last and self.health_tail.last["status"] == "fresh"), deadline, "depth_fresh")
        self.milestone("depth_observation_fresh")
        self.wait_for(lambda: self.sim_now() is not None, deadline, "native_clock")
        self.milestone("native_clock_observed")

    # ---- fixture scope ---------------------------------------------------------------------
    def fixture(self):
        from p3_fixtures import GROUND_WORLD, MODEL, generate, scene_change_pose
        px4 = self.root / "upstream/PX4-Autopilot"
        models = px4 / "Tools/simulation/gz/models"
        name = self.args.fixture
        (self.run / "world").mkdir()
        text, declaration = generate((px4 / "Tools/simulation/gz/worlds/default.sdf").read_text(), name,
                                     self.validation, self.sensors["optical_origin_model_m"][0])
        world_path = self.run / "world" / (GROUND_WORLD + ".sdf")
        world_path.write_text(text)
        self.summary.update(fixture=name, world=GROUND_WORLD, model=MODEL, fixture_declaration=declaration,
                            generated_world_sha256=digest(world_path))
        env = self.environment({"GZ_SIM_RESOURCE_PATH": str(models), "SDF_PATH": str(models),
                                "GZ_SIM_SYSTEM_PLUGIN_PATH": str(px4 / "build/px4_sitl_default_linux/src/modules/simulation/gz_plugins")
                                + ":" + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                                "GZ_SIM_SERVER_CONFIG_PATH": str(px4 / "src/modules/simulation/gz_bridge/server.config")})
        self.gazebo = self.launch("gazebo", ["gz", "sim", "-r", "-s", "--render-engine", "ogre2", "-v", "4",
                                             str(world_path)], env)
        watch = [b["name"] for b in declaration["boxes"]]
        self.sensor_stack(GROUND_WORLD, require_state=False, rgb=False, watch=watch)
        first = self.wait_for(self.sim_now, self.start + self.validation["startup_wall_s"], "fixture_clock")
        begin = first + self.validation["settle_sim_s"]
        length = (self.validation["fixtures"]["interruption"]["interrupt_after_sim_s"] if name == "interruption"
                  else self.validation["ground_window_sim_s"])
        self.summary["measurement_window_sim_s"] = [begin, begin + length]
        self.save()
        deadline = self.start + self.validation["fixture_wall_deadline_s"]
        self.wait_for(lambda: (self.sim_now() or 0) >= begin + length + 0.5, deadline, "measurement_window")
        if name == "interruption":
            # Deliberate fault: stop the only depth delivery path while recording continues.
            request_sim = self.sim_now()
            self.bridge.request_stop(signal.SIGINT)
            self.summary["interruption"] = {"component": "sensor-bridge", "signal": "SIGINT",
                                            "request_monotonic_s": self.bridge.record["stop_requested_monotonic_s"],
                                            "request_sim_s": request_sim}
            self.bridge.wait(self.validation["graceful_stop_wall_s"])
            from p3_process_exit import classify
            classification, reason = classify(self.bridge.record)
            self.summary["interruption"]["exit"] = {"code": self.bridge.record["exit_code"],
                                                    "classification": classification, "reason": reason}
            observe_until = time.monotonic() + self.validation["fixtures"]["interruption"]["observe_after_wall_s"]
            self.wait_for(lambda: time.monotonic() >= observe_until, observe_until + 1, "interruption_observation")
        if name == "plane4":
            pose = scene_change_pose(self.validation, self.sensors["optical_origin_model_m"][0])
            x, y, z = pose["position_m"]
            request = f'name: "front_plane", position: {{x: {x!r}, y: {y!r}, z: {z!r}}}, orientation: {{w: 1}}'
            response = subprocess.run(["gz", "service", "-s", f"/world/{GROUND_WORLD}/set_pose", "--reqtype",
                                       "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req",
                                       request], env=env, text=True, capture_output=True, timeout=10)
            applied = self.sim_now()
            self.summary["scene_change"] = {"requested_position_m": pose["position_m"], "applied_sim_s": applied,
                                            "response": response.stdout.strip(), "returncode": response.returncode}
            if response.returncode != 0 or "true" not in response.stdout:
                raise Abort("scene_change_request_failed")
            change = self.validation["scene_change"]
            self.wait_for(lambda: (self.sim_now() or 0) >= applied + change["settle_sim_s"] + change["window_sim_s"] + 0.5,
                          deadline, "scene_change_window")
        self.summary["status"] = "passed"

    # ---- PX4 stacks (readiness / flight) -------------------------------------------------------
    def px4_stack(self, frozen):
        from p1_runner import Console
        import mc_trace_runtime
        receipt = strict_json(self.root / "state/px4-mc-trace-built.json")
        selected = mc_trace_runtime.selected_build(self.root, SIM)
        if selected != frozen["runtime"]["mc_trace_build"]:
            raise Abort("mc_trace_build_differs_from_manifest")
        build = Path(receipt["build"])
        binary = Path(receipt["binary"])
        p2 = strict_json(self.root / "state/p2-built.json")
        qgc = strict_json(self.root / "state/qgc-unattended-built.json")
        config = strict_json(SIM / "configs/p2_control.yaml")
        timing = strict_json(SIM / "configs/p2_timing.yaml")
        work = self.run / "rootfs"
        work.mkdir()
        (work / "gz_env.sh").write_bytes((build / "rootfs/gz_env.sh").read_bytes())
        for name in ("p2_control.yaml", "p2_timing.yaml", "versions.lock.yaml"):
            shutil.copyfile(SIM / "configs" / name, self.run / name)
        self.summary.update(kind="flight" if self.scope == "flight" else "observe", config=config,
                            world="default", model=self.sensors["model"])
        self.summary["identity"].update(binary_sha256=digest(binary), mc_trace=receipt,
                                        p2_package_hash=p2["package_hash"],
                                        p2_config_sha256=digest(SIM / "configs/p2_control.yaml"))
        mc_env = mc_trace_runtime.prepare(self.run, receipt)
        # Native MAVLink capture starts before QGC so the startup window is complete.
        from p3_mavlink import LinkCapture
        self.capture = LinkCapture(self.run / "mavlink")
        self.summary["startup"]["origin_monotonic_ns"] = self.capture.started_ns
        qenv = {k: v for k, v in self.environment().items() if k in ("HOME", "PATH", "TMPDIR", "DISPLAY",
                                                                     "WAYLAND_DISPLAY", "XDG_RUNTIME_DIR")}
        qenv.update({"XDG_CONFIG_HOME": str(self.run / "qgc-config"), "XDG_DATA_HOME": str(self.run / "qgc-data"),
                     "XDG_CACHE_HOME": str(self.run / "qgc-cache"), "QT_QPA_PLATFORM": "offscreen",
                     "QT_QUICK_BACKEND": "software", "GWM_QGC_SHUTDOWN_MODE": "isolated-sitl-active-connection-v1",
                     "GWM_QGC_SITL_SHUTDOWN_RECEIPT": str(self.run / "qgc-shutdown-ownership.json"),
                     "GWM_QGC_AUDIT_NATIVE_ATTEMPTS": "v1"})
        (self.run / "qgc-config/QGroundControl").mkdir(parents=True)
        (self.run / "qgc-config/QGroundControl/QGroundControl.ini").write_text(
            (SIM / "configs/qgc-monitor.ini").read_text().replace("@RUN_DATA@", str(self.run / "qgc-data")))
        self.qgc = self.launch("qgc-monitor", [str(Path(qgc["bundle"]) / "squashfs-root/AppRun")], qenv,
                               stop_method="SIGTERM", grace=self.validation["qgc_graceful_stop_wall_s"], cwd=work)
        # P3 agent: pinned library, main() with a graceful SIGINT/SIGTERM stop (build_p3_agent.py).
        agent = strict_json(self.root / "state/p3-dds-agent-built.json")
        if agent.get("status") != "built" or digest(agent["binary"]) != agent["binary_sha256"]:
            raise Abort("p3_dds_agent_build_invalid")
        self.summary["identity"]["dds_agent"] = {"binary_sha256": agent["binary_sha256"],
                                                 "p3_main_source_sha256": agent["p3_main_source_sha256"]}
        denv = self.environment({"LD_LIBRARY_PATH": agent["library_dir"] + ":" + os.environ.get("LD_LIBRARY_PATH", "")})
        self.launch("dds-agent", [agent["binary"], "udp4", "-p", "8888", "-v", "4"], denv)
        penv = self.environment({"PX4_SIM_MODEL": self.sensors["airframe"], "PX4_GZ_WORLD": "default",
                                 "PX4_UXRCE_DDS_NS": "px4_71", "PX4_UXRCE_DDS_PORT": "8888",
                                 "PX4_PARAM_UXRCE_DDS_SYNCT": "0", "PX4_PARAM_SDLOG_MODE": "1", "HEADLESS": "1",
                                 **mc_env})
        self.px4_log = (self.run / "px4-console.log").open("x")
        self.streams.append(self.px4_log)
        command = [str(binary), "-i", "71", "-w", str(work), str(build / "etc")]
        self.console = Console(command, work, penv, self.px4_log, self.start + config["wall_deadline_s"] + 90)
        px4 = Owned("px4", self.console.process.pid, command, "console_shutdown", self.validation["graceful_stop_wall_s"],
                    self.console.process)
        self.owned.append(px4)
        self.summary["processes"].append(px4.record)
        self.px4 = px4
        boot = self.console.prompt(90)
        if "Gazebo world is ready" not in boot or self.sensors["model"] not in boot:
            raise Abort("expected_owned_world_or_depth_model_not_observed")
        self.milestone("px4_booted")
        self.gazebo = self.adopt("gazebo", lambda c: " sim " in f" {c} " and "default.sdf" in c and "gz" in c)
        self.save()
        for command in ("param show UXRCE_DDS_SYNCT", "uxrce_dds_client status", "logger status", "mc_pos_control status"):
            (self.run / ("console-" + command.replace(" ", "_") + ".txt")).write_text(self.console.command(command))
        env = self.environment()
        self.launch("clock-bridge", [PARAMETER_BRIDGE, "--ros-args", "-p",
                                     "config_file:=" + str(SIM / "configs/p2_clock_bridge.yaml"), "-p", "use_sim_time:=true"], env)
        self.sensor_stack("default", require_state=True, rgb=True, watch=[])
        from gwm_px4_control.execution_timing import controller_transport_env
        cenv = self.environment(controller_transport_env(timing))
        if self.scope == "flight":
            cenv["GWM_ALLOW_SITL_COMMANDS"] = "1"
        cenv["GWM_P2_OWNED_NAMESPACE"] = "1"
        controller = [str(Path(p2["install"]) / "gwm_px4_control/lib/gwm_px4_control/p2_control"),
                      "--config", str(self.run / "p2_control.yaml"), "--run-dir", str(self.run),
                      "--timing-config", str(self.run / "p2_timing.yaml")] + (["--flight"] if self.scope == "flight" else [])
        self.controller = self.launch("ros-controller", controller, cenv, stop_method="completes_own_work")
        self.summary["controller_transport_environment"] = controller_transport_env(timing)
        (self.run / "processes.txt").write_text(json.dumps(namespace_processes(), indent=1))
        return config

    def run_controller(self, config):
        deadline = self.start + config["wall_deadline_s"] + 90
        self.wait_for(lambda: self.controller.poll() is not None, deadline, "controller")
        result_path = self.run / "controller-result.json"
        self.summary["controller_exit_code"] = self.controller.record["exit_code"]
        self.summary["controller_result"] = strict_json(result_path) if result_path.exists() else None
        result = self.summary["controller_result"]
        if result is not None:
            self.summary["controller_publishers_only_from_controller"] = bool(result.get("graph_valid"))
        if self.controller.record["exit_code"] != 0:
            raise Abort("controller_failed")
        transitions = {t["to"]: t["sim_s"] for t in result["transitions"]}
        if self.scope == "flight":
            self.summary["sensor_window_sim_s"] = [transitions["PRESTREAM_SAFE_SETPOINTS"], transitions["COMPLETE"]]
        else:
            end = self.sim_now()
            self.summary["sensor_window_sim_s"] = [end - self.validation["readiness_observation_sim_s"], end]
        self.summary["status"] = "passed"

    def ground_state(self):
        status = self.console.topic("vehicle_status")
        landed = self.console.topic("vehicle_land_detected")
        return {"status": status, "landed": landed, "end_wall_s": time.monotonic()}

    def wait_landed_disarmed(self, limit):
        end = time.monotonic() + limit
        while True:
            self.pump()
            ground = self.ground_state()
            if ground["status"].get("arming_state") == 1 and ground["landed"].get("landed") is True:
                return ground
            if time.monotonic() > end:
                return None
            time.sleep(0.5)

    # ---- ordered shutdown -------------------------------------------------------------------
    def stop(self, name, sig=signal.SIGINT):
        item = next((o for o in self.owned if o.name == name), None)
        if item is None:
            return
        if item.poll() is None:
            item.request_stop(sig)
            if not item.wait(item.record["grace_s"]):
                item.force()

    def shutdown(self):
        errors = []

        def attempt(label, action):
            try:
                action()
            except Exception as exc:  # recorded; later drains still run
                errors.append(label + ":" + type(exc).__name__ + ":" + str(exc))

        self.shutting_down = True
        ground = None
        if self.console is not None and self.console.process.poll() is None:
            limit = strict_json(SIM / "configs/p2_control.yaml")["recovery_wall_s"]
            try:
                ground = self.wait_landed_disarmed(limit)
            except Exception as exc:
                errors.append("final_landed_disarmed_wait:" + type(exc).__name__ + ":" + str(exc))
            self.summary["final_console_state"] = ground
            if ground is None:
                errors.append("final_landed_disarmed_not_independently_observed")
        if self.scope != "fixture" and ground is not None:
            attempt("qgc_unattended_teardown", lambda: self.stop_qgc(ground))
        if hasattr(self, "controller") and self.controller.poll() is None:
            attempt("controller_stop", lambda: self.stop("ros-controller"))
        for name in ("sensor-bridge", "sensor-adapter", "source-probe", "clock-bridge"):
            attempt("stop:" + name, lambda n=name: self.stop(n))
        if self.console is not None and self.console.process.poll() is None:
            if ground is not None:
                attempt("mc_trace_drain", self.drain_mc)
                attempt("logger_stop", lambda: self.console.command("logger stop"))
            self.px4.mark_console_stop()
            attempt("px4_shutdown", lambda: os.write(self.console.fd, b"shutdown\n"))
            if not self.px4.wait(self.validation["graceful_stop_wall_s"]):
                self.px4.force()
        for name in ("qgc-monitor", "dds-agent"):
            attempt("stop:" + name, lambda n=name: self.stop(n, signal.SIGTERM if n == "qgc-monitor" else signal.SIGINT))
        if hasattr(self, "gazebo"):
            attempt("gazebo_pause", self.pause_world)
            attempt("stop:gazebo", lambda: self.stop("gazebo"))
        for item in self.owned:
            if item.poll() is None:
                item.force()
        # PX4 spawns Gazebo into its own process group; Gazebo is owned and judged by its own
        # record. Only a member that is itself an owned, reaped process is set aside (raw kept).
        reaped = {o.pid for o in self.owned if o.record["reaped"]}
        for item in self.owned:
            raw = item.record["remaining_group_members"] or []
            separate = [pid for pid in raw if pid in reaped and pid != item.pid]
            if separate:
                item.record.update(group_members_at_exit_raw=raw, group_members_owned_separately=separate,
                                   remaining_group_members=[pid for pid in raw if pid not in separate])
        if self.capture is not None:
            attempt("mavlink_capture_close", self.capture.close)
        residual = [row for row in namespace_processes() if row["state"] != "Z"]
        self.summary["residual_namespace_processes"] = residual
        for row in residual:
            try:
                os.kill(row["pid"], signal.SIGKILL)
            except ProcessLookupError:
                pass
        return errors

    def stop_qgc(self, ground):
        from qgc_unattended import ownership_receipt, require_teardown
        self.sim_now()  # refresh the independent native clock observation
        clock = {"progress_wall_s": (self.clock_tail.last or {}).get("receipt_monotonic_ns", 0) / 1e9, "error": None}
        ground = self.ground_state()
        owner = SIM / "scripts/p3_namespace_owner.py"
        ownership_receipt(self.run, self.qgc.popen, ground, clock, owner, digest(owner))
        self.stop("qgc-monitor", signal.SIGTERM)
        record = self.qgc.record
        cleanup = {"exit_code": record["exit_code"], "forced": record["forced"],
                   "registered_members_after": record["remaining_group_members"]}
        self.summary["qgc_normal_teardown"] = require_teardown(self.run / "qgc-monitor.log", cleanup)

    def drain_mc(self):
        self.console.command("mc_pos_control stop")
        marker = self.run / "mc-trace.bin.closed"
        end = time.monotonic() + 10
        while not marker.exists():
            if time.monotonic() > end:
                raise Abort("mc_trace_drain_timeout")
            time.sleep(0.05)
        import mc_trace_runtime
        mc_trace_runtime.finalize(self.run, self.summary)

    def pause_world(self):
        world = self.summary.get("world")
        result = subprocess.run(["gz", "service", "-s", f"/world/{world}/control", "--reqtype", "gz.msgs.WorldControl",
                                 "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", "pause: true"],
                                env=self.environment(), text=True, capture_output=True, timeout=10)
        self.summary["gazebo_pause_before_stop"] = {"returncode": result.returncode, "stdout": result.stdout.strip()}

    # ---- finalization ----------------------------------------------------------------------
    def renderer(self):
        log = Path.home() / ".gz/rendering/ogre2.log"
        result = {"identified": False, "source": str(log)}
        if log.exists() and log.stat().st_mtime >= self.summary["started_unix_ns"] / 1e9:
            shutil.copyfile(log, self.run / "ogre2.log")
            text = (self.run / "ogre2.log").read_text(errors="replace")
            found = re.findall(r"GL_RENDERER\s*=\s*(.+)", text) + re.findall(r"GL_VENDOR\s*=\s*(.+)", text)
            result.update(strings=sorted(set(s.strip() for s in found)), sha256=digest(self.run / "ogre2.log"))
            result["software"] = any(SOFTWARE_RENDERERS.search(s) for s in result["strings"])
            result["identified"] = bool(found) and not result["software"]
        return result

    def finalize(self):
        if self.sampler is not None:
            self.sampler.stop_event.set()
            self.sampler.join(timeout=3)
            self.summary["resources"] = {"peaks": self.sampler.peaks}
        for stream in self.streams:
            try:
                stream.close()
            except OSError:
                pass
        if self.console is not None:
            os.close(self.console.fd)
        work = self.run / "rootfs"
        self.summary["ulog_files"] = [{"path": str(p), "bytes": p.stat().st_size, "sha256": digest(p)}
                                      for p in work.rglob("*.ulg")] if work.exists() else []
        self.summary["rosbag_files"] = [{"path": str(p), "bytes": p.stat().st_size, "sha256": digest(p)}
                                        for p in (self.run / "rosbag").glob("*") if p.is_file()]
        self.summary["renderer"] = self.renderer()
        self.summary["disk_free_after_bytes"] = shutil.disk_usage(self.root).free
        self.summary["wall_duration_s"] = time.monotonic() - self.start
        artifacts = []
        for path in sorted(self.run.rglob("*")):
            if path.is_file() and path.name not in ("summary.json",) and not path.name.startswith("summary.json.tmp"):
                artifacts.append({"path": path.relative_to(self.run).as_posix(), "bytes": path.stat().st_size,
                                  "sha256": digest(path)})
        self.summary["artifacts"] = artifacts
        self.summary["run_bytes"] = sum(a["bytes"] for a in artifacts)
        self.summary["finalized"] = True
        self.save()

    # ---- entry -----------------------------------------------------------------------------
    def execute(self):
        frozen, sha, authorization = self.preconditions()
        label = self.args.fixture if self.scope == "fixture" else self.scope
        self.run = self.root / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-p3-" + label + "-"
                                         + uuid.uuid4().hex[:8])
        self.run.mkdir(parents=True)
        (self.run / "tmp").mkdir()
        for name in ("p3_sensors.yaml", "p3_validation.yaml", "p3_sensor_bridge.yaml", "p3_fastdds.xml",
                     "p3_native_command_policy.json"):
            shutil.copyfile(SIM / "configs" / name, self.run / name)
        print("P3 evidence: " + str(self.run), flush=True)
        self.summary = {"schema_version": 1, "contract": "p3-runtime-summary-v1", "run_id": self.run.name,
                        "p3_kind": self.scope, "gate": self.args.gate, "role": self.args.role,
                        "frozen_manifest_sha256": sha, "authorization_sha256": digest(self.args.authorization),
                        "started_unix_ns": time.time_ns(), "status": "incomplete", "failure": None,
                        "identity": {}, "processes": [], "startup": {"milestones": [], "origin_monotonic_ns": None},
                        "native_command_policy": frozen["runtime"]["native_command_policy"],
                        "project_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=SIM, text=True).strip(),
                        "isolation": "private user/mount/PID/network namespace, loopback only, exclusive workspace lock",
                        "flight_commands_possible": self.scope == "flight", "finalized": False}
        self.sampler = ResourceSampler(self.run, self.owned, self.root)
        self.sampler.start()
        try:
            self.save()
            if self.scope == "fixture":
                self.fixture()
            else:
                config = self.px4_stack(frozen)
                self.run_controller(config)
        except (Exception, KeyboardInterrupt) as exc:
            self.summary["status"] = "failed"
            self.summary["failure"] = type(exc).__name__ + ":" + str(exc)
            print("P3 stopped: " + self.summary["failure"], flush=True)
        finally:
            try:
                errors = self.shutdown()
            except Exception as exc:
                errors = ["shutdown_exception:" + type(exc).__name__ + ":" + str(exc)]
            self.summary["shutdown_errors"] = errors
            if errors and self.summary["status"] == "passed":
                self.summary["status"], self.summary["failure"] = "failed", "shutdown:" + errors[0]
            self.finalize()
        print(json.dumps({"run_id": self.run.name, "status": self.summary["status"],
                          "failure": self.summary["failure"]}), flush=True)
        return 0 if self.summary["status"] == "passed" else 1


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true", required=True)
    parser.add_argument("--scope", choices=("readiness", "fixture", "flight"), required=True)
    parser.add_argument("--fixture")
    parser.add_argument("--gate", choices=("A", "B", "C"), required=True)
    parser.add_argument("--role", required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    args = parser.parse_args(argv)
    if (args.scope == "fixture") != bool(args.fixture):
        parser.error("--fixture is required exactly for the fixture scope")
    return Runner(args).execute()


if __name__ == "__main__":
    raise SystemExit(main())
