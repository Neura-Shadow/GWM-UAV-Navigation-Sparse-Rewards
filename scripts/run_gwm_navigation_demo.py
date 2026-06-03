"""CLI for the mock-first end-to-end GWM navigation demo."""

from __future__ import annotations

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from src.generated_world_model.demo import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())

