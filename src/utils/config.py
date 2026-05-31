"""YAML-based configuration loader with deep-merge support.

Usage::

    from src.utils.config import load_config, merge_configs

    base = load_config("configs/default.yaml")
    override = load_config("configs/experiment.yaml")
    cfg = merge_configs(base, override)
"""

import copy
import logging
from pathlib import Path
from typing import Any, Dict

import yaml

logger = logging.getLogger("gwm_uav.config")


def load_config(path: str) -> Dict[str, Any]:
    """Load a YAML configuration file and return it as a dictionary.

    Args:
        path: Filesystem path to a ``.yaml`` / ``.yml`` file.

    Returns:
        Parsed configuration dictionary.

    Raises:
        FileNotFoundError: If *path* does not point to an existing file.
        yaml.YAMLError: If the file contains invalid YAML.
    """
    config_path = Path(path)
    if not config_path.is_file():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    logger.info("Loading configuration from %s", config_path)
    with open(config_path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    if data is None:
        logger.warning("Configuration file %s is empty, returning empty dict", config_path)
        return {}

    return data


def merge_configs(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Deep-merge *override* into a copy of *base*.

    Nested dictionaries are merged recursively; all other values in
    *override* replace those in *base*.

    Args:
        base: Base configuration dictionary.
        override: Override dictionary whose values take precedence.

    Returns:
        A new merged dictionary (neither input is mutated).
    """
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if (
            key in merged
            and isinstance(merged[key], dict)
            and isinstance(value, dict)
        ):
            merged[key] = merge_configs(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged
