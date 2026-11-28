"""P8 namespace-owner entry shim (not the P3 runner).

`p8_namespace_owner.py` is a byte-identical copy of the P3 owner, whose SHA-256 the unattended QGC build pins. Its
code does `from p3_runner import main` from its own directory, which resolves here and dispatches to the P8 runner.
The accepted P7-E runner, and through it the frozen P7 runner and the accepted P5, P4 and P3 runners, are imported
by p8_runner.py from their own paths.
"""
from p8_runner import main  # noqa: F401
