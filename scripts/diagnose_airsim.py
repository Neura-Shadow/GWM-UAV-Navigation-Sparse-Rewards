#!/usr/bin/env python3
"""AirSim / CosysAirSim diagnostic script.

Checks connectivity, settings, vehicles, and LiDAR sensor availability.
This is a refactored version of the original ``main2.py`` with English
comments, proper logging, and argparse support.

Usage
-----
    # Auto-detect host (WSL gateway) with default port:
    python scripts/diagnose_airsim.py

    # Specify host and port explicitly:
    python scripts/diagnose_airsim.py --host 192.168.1.100 --port 41451
"""

from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
from typing import List, Optional

logger = logging.getLogger("diagnose_airsim")

# ---- Import guard ----
try:
    import cosysairsim as airsim
except ImportError:
    try:
        import airsim  # type: ignore[no-redef]
    except ImportError:
        airsim = None  # type: ignore[assignment]


# ------------------------------------------------------------------
# Host detection
# ------------------------------------------------------------------

def get_wsl_host_ip() -> str:
    """Attempt to detect the Windows host IP when running inside WSL2.

    Strategy:
    1. Parse ``ip route show default`` for the gateway address.
    2. Fall back to the nameserver in ``/etc/resolv.conf``.
    3. Return ``127.0.0.1`` if both methods fail (native Windows).
    """
    # Method 1: default gateway (WSL2 convention)
    try:
        output = subprocess.check_output(
            ["ip", "route", "show", "default"], text=True
        )
        for line in output.splitlines():
            parts = line.split()
            if "via" in parts:
                ip = parts[parts.index("via") + 1]
                logger.debug("WSL host IP from default route: %s", ip)
                return ip
    except Exception:
        pass

    # Method 2: resolv.conf nameserver
    try:
        with open("/etc/resolv.conf", "r") as f:
            for line in f:
                if "nameserver" in line:
                    ip = line.split()[1]
                    logger.debug("WSL host IP from resolv.conf: %s", ip)
                    return ip
    except Exception:
        pass

    return "127.0.0.1"


# ------------------------------------------------------------------
# Diagnostics
# ------------------------------------------------------------------

def check_settings(client: "airsim.MultirotorClient") -> None:
    """Retrieve and inspect the active UE settings string."""
    if not hasattr(client, "getSettingsString"):
        logger.warning(
            "This airsim build does not support getSettingsString()."
        )
        return

    try:
        settings_text = client.getSettingsString()
    except Exception as err:
        logger.error("Failed to retrieve UE settings: %s", err)
        return

    has_sensors = '"Sensors"' in settings_text
    has_lidar_sensor1 = "LidarSensor1" in settings_text
    has_lidar_key = '"Lidar"' in settings_text
    has_lidar1 = "Lidar1" in settings_text

    logger.info(
        "Settings check:  Sensors=%s  LidarSensor1=%s  Lidar=%s  Lidar1=%s",
        has_sensors, has_lidar_sensor1, has_lidar_key, has_lidar1,
    )

    if not (has_sensors or has_lidar_key):
        logger.warning(
            "No sensor/LiDAR configuration detected in the active settings!"
        )
        logger.info("Active settings excerpt:\n%s", settings_text[:400].strip())
        logger.info(
            "Hint: launch UnrealEditor with the -settings flag pointing to "
            "your settings.json, e.g.:\n"
            '  UnrealEditor.exe "D:\\...\\UAVSIM.uproject" '
            '-settings="C:\\Users\\...\\AirSim\\settings.json"'
        )


def list_vehicles(client: "airsim.MultirotorClient") -> List[str]:
    """Query UE for the list of spawned vehicles."""
    try:
        vehicles = client.listVehicles()
        logger.info("Vehicles reported by UE: %s", vehicles)
        return vehicles
    except Exception:
        logger.debug("listVehicles() not supported or failed.")
        return []


def select_vehicle(
    env_vehicle: str,
    available: List[str],
    default: str = "SimpleFlight",
) -> str:
    """Choose which vehicle name to use for subsequent API calls."""
    if env_vehicle:
        return env_vehicle
    if available:
        return available[0]
    return default


def try_arm_vehicle(
    client: "airsim.MultirotorClient",
    vehicle: str,
    available: List[str],
) -> str:
    """Enable API control and arm the vehicle, falling back if needed."""
    try:
        client.enableApiControl(True, vehicle)
        client.armDisarm(True, vehicle)
        logger.info("Armed vehicle: %s", vehicle)
        return vehicle
    except Exception as err:
        if available and vehicle != available[0]:
            fallback = available[0]
            logger.warning(
                "Vehicle '%s' unavailable (%s). Falling back to '%s'.",
                vehicle, err, fallback,
            )
            client.enableApiControl(True, fallback)
            client.armDisarm(True, fallback)
            return fallback
        raise


def test_lidar(
    client: "airsim.MultirotorClient",
    vehicle: str,
    preferred_lidar: str,
) -> None:
    """Try multiple LiDAR sensor names until one succeeds."""
    candidates = [preferred_lidar, "LidarSensor1", "Lidar1", "Lidar"]
    # Deduplicate while preserving order
    seen = set()
    unique: List[str] = []
    for name in candidates:
        if name not in seen:
            seen.add(name)
            unique.append(name)

    last_error: Optional[Exception] = None

    for lidar_name in unique:
        try:
            data = client.getLidarData(lidar_name, vehicle)
            logger.info(
                "LiDAR OK — vehicle=%s  sensor=%s  points=%d",
                vehicle, lidar_name, len(data.point_cloud),
            )
            return
        except Exception as err:
            last_error = err
            logger.debug("LiDAR '%s' failed: %s", lidar_name, err)

    # Last resort: empty-string sensor name
    try:
        data = client.getLidarData("", vehicle)
        logger.info(
            "LiDAR OK (default) — vehicle=%s  points=%d",
            vehicle, len(data.point_cloud),
        )
        return
    except Exception as err:
        last_error = err

    logger.error(
        "All LiDAR attempts failed.\n"
        "  Vehicle:  %s\n"
        "  Tried:    %s + ['<default>']\n"
        "  Last err: %s\n"
        "This is usually caused by a mismatch between SettingsVersion "
        "and the LiDAR block format.",
        vehicle, unique, last_error,
    )


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------

def run_diagnostics(host: str, port: int) -> None:
    """Run the full AirSim diagnostic sequence."""
    if airsim is None:
        logger.error(
            "Neither cosysairsim nor airsim is installed. "
            "Install with:  pip install cosysairsim  or  pip install airsim"
        )
        sys.exit(1)

    client = airsim.MultirotorClient(ip=host, port=port)

    try:
        logger.info("Connecting to AirSim at %s:%d …", host, port)
        client.confirmConnection()
        logger.info("Connection successful!")
    except Exception as err:
        logger.error(
            "Connection failed.\n"
            "  Target:  %s:%d\n"
            "  Error:   %s\n"
            "Make sure UE with AirSim is running and the ApiServerPort is correct.",
            host, port, err,
        )
        sys.exit(1)

    # 1. Settings inspection
    check_settings(client)

    # 2. Vehicle listing
    available = list_vehicles(client)

    # 3. Vehicle selection and arming
    env_vehicle = os.environ.get("AIRSIM_VEHICLE", "")
    chosen = select_vehicle(env_vehicle, available)
    logger.info("Selected vehicle: %s", chosen)
    chosen = try_arm_vehicle(client, chosen, available)

    # 4. LiDAR testing
    preferred_lidar = os.environ.get("AIRSIM_LIDAR", "Lidar1")
    test_lidar(client, chosen, preferred_lidar)

    logger.info("Diagnostics complete.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Diagnose AirSim / CosysAirSim connectivity, settings, and sensors.",
    )
    parser.add_argument(
        "--host",
        default=None,
        help=(
            "AirSim host IP. Defaults to AIRSIM_HOST env var, "
            "then WSL gateway auto-detection, then 127.0.0.1."
        ),
    )
    parser.add_argument(
        "--port",
        type=int,
        default=41451,
        help="AirSim API port (default: 41451).",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable DEBUG-level logging.",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    host = args.host or os.environ.get("AIRSIM_HOST", "") or get_wsl_host_ip()
    run_diagnostics(host, args.port)


if __name__ == "__main__":
    main()
