"""P5 namespace-owner entry shim (not the P3 runner).

The unattended QGC build pins the SHA-256 of the namespace-owner script that runs as PID 1.
`p5_namespace_owner.py` is therefore a byte-identical copy of the P3 owner, whose code does
`from p3_runner import main` from its own directory. In the P5 tree that import resolves here and
dispatches to the P5 runner. The accepted P4 runner (and through it the P3 runner) is imported by
p5_runner.py from its own path, never through this module.
"""
from p5_runner import main  # noqa: F401
