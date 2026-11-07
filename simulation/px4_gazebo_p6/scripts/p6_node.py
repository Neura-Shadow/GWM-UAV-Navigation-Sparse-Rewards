"""Entry point of one P6 injector process: `p6_node.py <module> <arguments>`.

Runs `injection.<module>.main(arguments)` from the manifest-bound P6 tree. The runner starts it with
`/usr/bin/python3 -s -P`, so neither the user site nor the working directory is on `sys.path`.
"""
import importlib
from pathlib import Path
import sys

P6 = Path(__file__).resolve().parents[1]
MODULES = ("depth_relay", "state_relay", "witness", "competing_publisher", "controller_shim")

if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in MODULES:
        raise SystemExit("usage: p6_node.py {%s} ..." % "|".join(MODULES))
    sys.path.insert(0, str(P6))
    raise SystemExit(importlib.import_module("injection." + sys.argv[1]).main(sys.argv[2:]))
