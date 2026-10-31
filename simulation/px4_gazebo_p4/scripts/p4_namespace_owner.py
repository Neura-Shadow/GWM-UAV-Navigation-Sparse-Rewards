"""Stable PID-1 owner of one private P3 namespace.

This tiny file is the owner identity compiled into the unattended-shutdown QGC
build (its SHA-256 is the QGC guard's expected owner hash), so runner changes
do not require a QGC rebuild. The complete runner, configuration, evaluator
and binary identity is bound separately by the frozen P3 candidate manifest,
which the runner re-verifies before launching anything.
"""
from pathlib import Path
import sys

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from p3_runner import main
    raise SystemExit(main(sys.argv[1:]))
