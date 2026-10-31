"""Build the P3 DDS agent executable with a graceful SIGINT/SIGTERM stop; never starts PX4.

Only main() is replaced (validation/dds_agent_sigstop/microxrce_agent.cpp); it links the
unchanged pinned libmicroxrcedds_agent from dds-install, which accepted P2 keeps using.
The smoke test runs the agent alone in a private user/network namespace and requires
exit 0 after each stop signal.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / "validation"))
from p3_provenance import atomic_json, digest  # noqa: E402

COMMIT = "73622810d984349b80bbac0ef55fc0b694d62222"
UPSTREAM_MAIN_SHA256 = "c3bdc86a7b6bcfdecdd916ab593a92d0a0bebdb1451010868095814f45c9431e"
SOURCE = SIM / "validation/dds_agent_sigstop/microxrce_agent.cpp"


def smoke(binary, env, sig):
    """Agent alone, loopback only; exit code after one stop signal."""
    process = subprocess.Popen(["unshare", "--user", "--map-current-user", "--net", "--",
                                str(binary), "udp4", "-p", "18888", "-v", "4"],
                               env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(2.0)
    process.send_signal(sig)
    try:
        output, _ = process.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate()
        return {"signal": sig.name, "exit_code": None, "timed_out": True, "output_tail": output[-800:]}
    return {"signal": sig.name, "exit_code": process.returncode, "timed_out": False, "output_tail": output[-800:]}


def build(root):
    import signal
    upstream = root / "upstream/Micro-XRCE-DDS-Agent"
    install = root / "dds-install"
    if subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=upstream, text=True).strip() != COMMIT:
        raise ValueError("DDS agent checkout is not the pinned commit")
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=upstream, text=True).strip():
        raise ValueError("DDS agent checkout is dirty")
    if digest(upstream / "microxrce_agent.cpp") != UPSTREAM_MAIN_SHA256:
        raise ValueError("Pinned upstream main differs from the reviewed basis")
    source_sha = digest(SOURCE)
    folder = root / "p3_agent" / source_sha[:16]
    binary = folder / "bin/MicroXRCEAgent"
    if folder.exists():
        raise ValueError("P3 agent build folder already exists: " + str(folder))
    # Same defines, includes, flags and link inputs as the pinned MicroXRCEAgent target.
    superbuild = upstream / "build-jazzy-upstream-logger"
    spdlog = superbuild / "temp_install/spdlog-1.9.2"
    staging = root / "p3_agent" / (".building-" + source_sha[:16])
    staging.mkdir(parents=True, exist_ok=False)
    command = ["/usr/bin/c++", "-DFASTCDR_DYN_LINK", "-DSPDLOG_COMPILED_LIB", "-I" + str(install / "include"),
               "-isystem", "/opt/ros/jazzy/include/fastcdr", "-isystem", str(spdlog / "include"),
               "-O3", "-DNDEBUG", "-std=gnu++11", str(SOURCE), "-o", str(staging / "MicroXRCEAgent"),
               str(install / "lib/libmicroxrcedds_agent.so.2.4.3"), "-lrt", "-ldl", str(spdlog / "lib/libspdlog.a"),
               "/opt/ros/jazzy/lib/libfastcdr.so.2.2.8", "-Wl,-rpath-link,/opt/ros/jazzy/lib:" + str(install / "lib")]
    subprocess.run(command, check=True)
    binary.parent.mkdir(parents=True)
    (staging / "MicroXRCEAgent").rename(binary)
    staging.rmdir()
    env = dict(os.environ, LD_LIBRARY_PATH=str(install / "lib") + ":" + os.environ.get("LD_LIBRARY_PATH", ""))
    help_run = subprocess.run([str(binary), "--help"], env=env, capture_output=True, text=True, timeout=30)
    results = [smoke(binary, env, signal.SIGINT), smoke(binary, env, signal.SIGTERM)]
    linked = subprocess.run(["ldd", str(binary)], env=env, capture_output=True, text=True).stdout
    libraries = {str(p): digest(p) for p in sorted((install / "lib").glob("libmicro*.so*")) if p.is_file()}
    passed = (help_run.returncode == 1 and "Usage: 'MicroXRCEAgent" in help_run.stdout + help_run.stderr
              and all(r["exit_code"] == 0 for r in results)
              and str(install / "lib/libmicroxrcedds_agent.so") in linked)
    receipt = {"contract": "p3-dds-agent-sigstop-v1", "status": "built" if passed else "failed",
               "upstream_commit": COMMIT, "upstream_main_sha256": UPSTREAM_MAIN_SHA256,
               "p3_main_source_sha256": source_sha, "compile_command": command,
               "binary": str(binary), "binary_sha256": digest(binary), "library_dir": str(install / "lib"),
               "pinned_libraries": libraries, "help_exit_code": help_run.returncode, "stop_smoke": results,
               "ldd": linked, "graceful_stop": "sigwait(SIGINT|SIGTERM) -> AgentInstance::stop() -> exit 0"}
    atomic_json(root / "state/p3-dds-agent-built.json", receipt, exclusive=True)
    if not passed:
        raise SystemExit("P3 DDS agent build or stop smoke failed: " + str(root / "state/p3-dds-agent-built.json"))
    print("P3 DDS agent built: " + receipt["binary_sha256"], flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true")
    if parser.parse_args().build:
        build(Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy"))))
    else:
        print("No build. Use --build.")
