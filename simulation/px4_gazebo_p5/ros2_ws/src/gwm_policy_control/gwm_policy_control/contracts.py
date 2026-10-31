"""Strict P5 configuration and model-schema contract (p5-acceptance-v1 revision 1, §3–§5)."""
import hashlib
import json
from pathlib import Path

POLICY_CONTRACT = "p5-policy-config-v1"
SCHEMA_CONTRACT = "p5-model-schema-v1"
ARCHITECTURE = "p5-ranker-mlp-v1"
STATES = ("NORMAL", "OBSTACLE_DETECTED", "AVOIDING", "RESUME", "HOVER")
SLOTS = 26
CANDIDATE_FEATURES = 11
CONTEXT_FEATURES = 35
FEATURES = CANDIDATE_FEATURES + CONTEXT_FEATURES
LAYER_SHAPES = {"W1": (46, 64), "b1": (64,), "W2": (64, 64), "b2": (64,), "W3": (64, 1), "b3": (1,)}


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def load_schema(path):
    schema = json.loads(Path(path).read_text(encoding="utf-8"))
    if schema.get("schema_version") != 1 or schema.get("contract") != SCHEMA_CONTRACT:
        raise ValueError("Unsupported P5 model schema")
    if schema.get("architecture") != ARCHITECTURE or schema.get("dtype") != "float64":
        raise ValueError("P5 model schema architecture or dtype changed")
    slots = schema["slots"]
    if (slots["count"] != SLOTS or slots["heading_steps"] != 6 or slots["lengths_m"] != [0.75, 1.5]
            or len(schema["candidate_features"]) != CANDIDATE_FEATURES or schema["features_per_slot"] != FEATURES):
        raise ValueError("P5 model schema slot or feature layout changed")
    states = next(f for f in schema["context_features"] if f["name"] == "state_one_hot")["values"]
    if tuple(states) != STATES or schema["layers"] != [[46, 64], [64, 64], [64, 1]]:
        raise ValueError("P5 model schema states or layers changed")
    return schema


def schema_sha256(schema):
    return canonical_sha256(schema)


def load_policy_config(path):
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    if config.get("schema_version") != 1 or config.get("contract") != POLICY_CONTRACT:
        raise ValueError("Unsupported P5 policy configuration")
    if not isinstance(config.get("enabled"), bool):
        raise ValueError("P5 'enabled' must be an explicit boolean")
    if config.get("fallback") != "none" or config["model"].get("require_trained") is not True:
        raise ValueError("P5 contract value changed: acceptance builds have no fallback and require a trained model")
    if config["inference"] != {"failure_wall_s": 0.020, "consecutive_failures_to_abort": 3}:
        raise ValueError("P5 contract value changed: inference failure rule")
    if config["model"].get("architecture") != ARCHITECTURE:
        raise ValueError("P5 contract value changed: architecture")
    return config


def require_enabled(config, authorization_present):
    """Contract C6: nothing runs unless enabled by an authorized P5 run with a verified checkpoint."""
    model = config["model"]
    if config["enabled"] is not True or not authorization_present or not model.get("path") or not model.get("sha256"):
        raise PermissionError("P5 is disabled: requires enabled=true, a consumed P5 authorization and a bound checkpoint")
