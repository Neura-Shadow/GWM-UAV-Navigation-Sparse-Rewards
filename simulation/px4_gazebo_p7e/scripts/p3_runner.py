"""P7-E namespace-owner entry shim (not the P3 runner).

`p7e_namespace_owner.py` is a byte-identical copy of the P3 owner, whose SHA-256 the unattended QGC build pins. Its
code does `from p3_runner import main` from its own directory, which resolves here and dispatches to the P7-E runner.
The frozen P7 runner, and through it the accepted P5, P4 and P3 runners, are imported by p7e_runner.py from their
own paths.
"""
from p7e_runner import main  # noqa: F401
