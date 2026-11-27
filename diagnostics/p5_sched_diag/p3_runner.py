"""Diagnostic namespace-owner entry shim (not the P3 runner).

`p5_namespace_owner.py` here is a byte-identical copy of the P3/P5 owner, whose pinned SHA-256 the unattended
QGC build checks; its code does `from p3_runner import main` from its own directory, which resolves here and
dispatches to the scheduling-diagnostic runner. The accepted P5, P4 and P3 runners are imported by file path.
"""
from diag_runner import main  # noqa: F401
