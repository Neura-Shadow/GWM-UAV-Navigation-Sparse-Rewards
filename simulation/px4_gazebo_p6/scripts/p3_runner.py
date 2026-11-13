"""P6 namespace-owner entry shim (not the P3 runner).

`p6_namespace_owner.py` is a byte-identical copy of the P3 owner, whose SHA-256 the unattended QGC build
pins; its code does `from p3_runner import main` from its own directory, which resolves here and dispatches
to the P6 runner. The accepted P5, P4 and P3 runners are imported by p6_runner.py from their own paths.
"""
from p6_runner import main  # noqa: F401
