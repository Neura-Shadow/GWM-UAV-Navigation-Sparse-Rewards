"""Runtime-free validation for the operator's fixed simulation-only trial."""
import json
import math
from pathlib import Path

GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")


def require_gates(env, flight=False):
    required = GATES + (("GWM_ALLOW_SITL_COMMANDS",) if flight else ())
    missing = [key for key in required if env.get(key) != "1"]
    if missing:
        raise ValueError("Missing explicit process-local gates: " + ", ".join(missing))


def load_config(path):
    config = json.loads(Path(path).read_text())  # JSON is a YAML 1.2 subset.
    for key, expected in (("schema_version", 1), ("model", "x500"), ("world", "default"),
                          ("control_owner", "local_px4_console")):
        if config.get(key) != expected:
            raise ValueError("Unsupported trial configuration: " + key)
    for key in ("target_height_m", "hover_duration_sim_s", "height_tolerance_m",
                "horizontal_displacement_m", "max_sample_gap_sim_s",
                "simulation_deadline_s", "wall_deadline_s"):
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("Expected a finite positive number: " + key)
    if config["hover_duration_sim_s"] >= config["simulation_deadline_s"]:
        raise ValueError("Hover must fit inside simulation deadline")
    if config["height_tolerance_m"] >= config["target_height_m"]:
        raise ValueError("Ground must not satisfy hover tolerance")
    return config


def evaluate_hover(samples, config, ground_z):
    if len(samples) < 2:
        return {"status": "unknown", "reason": "insufficient samples"}
    if not isinstance(ground_z, (int, float)) or not math.isfinite(ground_z):
        return {"status": "unknown", "reason": "missing ground reference"}
    if any(k not in s or isinstance(s[k], bool) or not isinstance(s[k], (int, float))
           or not math.isfinite(s[k]) for s in samples for k in ("t", "x", "y", "z")):
        return {"status": "unknown", "reason": "non-finite sample"}
    gaps = [b["t"] - a["t"] for a, b in zip(samples, samples[1:])]
    heights = [ground_z - s["z"] for s in samples]
    error = max(abs(h - config["target_height_m"]) for h in heights)
    displacement = max(math.hypot(s["x"] - samples[0]["x"], s["y"] - samples[0]["y"]) for s in samples)
    duration = samples[-1]["t"] - samples[0]["t"]
    passed = (min(gaps) > 0 and max(gaps) <= config["max_sample_gap_sim_s"]
              and duration >= config["hover_duration_sim_s"]
              and error <= config["height_tolerance_m"]
              and displacement <= config["horizontal_displacement_m"])
    return {"status": "passed" if passed else "failed", "samples": len(samples),
            "start_sim_s": samples[0]["t"], "end_sim_s": samples[-1]["t"],
            "duration_sim_s": duration, "max_sample_gap_sim_s": max(gaps),
            "height_min_m": min(heights), "height_max_m": max(heights),
            "height_max_error_m": error, "horizontal_max_displacement_m": displacement}


def parse_topic(text):
    import re
    result = {}
    for key, value in re.findall(r"^\s*([a-zA-Z_][a-zA-Z_0-9]*):\s+([^\r\n]+)", text, re.MULTILINE):
        token = value.split()[0]
        if token in ("True", "False", "true", "false"):
            result[key] = token.lower() == "true"
        else:
            try:
                number = float(token)
                result[key] = number if math.isfinite(number) else token
            except ValueError:
                result[key] = token
    return result


def completed_console_response(text, command=None):
    """Ignore PX4's per-keystroke prompt redraws until the command finishes."""
    import re
    clean = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", text)
    response = clean
    if command is not None:
        # PX4 may append an asynchronous log between the final echoed
        # character and its newline. Still require that newline and a later
        # prompt; an incomplete redraw alone must never complete a command.
        echo = re.search(re.escape("pxh> " + command) + r"[^\r\n]*\r?\n", clean)
        if echo is None:
            return None
        response = clean[echo.end():]
    if re.search(r"(?:^|\n)pxh> ", response):
        return response
    return None
