"""Small mock OpenUSD stage objects for Isaac Sim scene-builder tests."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class MockUSDPrim:
    """Lightweight record of a USD prim definition."""

    path: str
    prim_type: str
    attributes: Dict[str, Any] = field(default_factory=dict)

    def set_attribute(self, name: str, value: Any) -> None:
        """Set an attribute value on this mock prim."""
        self.attributes[name] = value


class MockUSDStage:
    """Minimal mock of a USD stage.

    The API intentionally mirrors only the tiny surface needed by
    ``IsaacSimSceneBuilder`` tests: prim definition, prim lookup, and JSON
    export of the recorded stage.
    """

    def __init__(self) -> None:
        self.prims: Dict[str, MockUSDPrim] = {}
        self.define_log: List[Dict[str, str]] = []

    def DefinePrim(self, path: str, prim_type: str) -> MockUSDPrim:
        """Record and return a prim definition."""
        prim = MockUSDPrim(path=path, prim_type=prim_type)
        self.prims[path] = prim
        self.define_log.append({"path": path, "prim_type": prim_type})
        return prim

    def GetPrimAtPath(self, path: str) -> Optional[MockUSDPrim]:
        """Return a previously defined prim, if present."""
        return self.prims.get(path)

    def Export(self, path: str) -> None:
        """Write the mock stage contents as JSON."""
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "prims": [
                {
                    "path": prim.path,
                    "prim_type": prim.prim_type,
                    "attributes": prim.attributes,
                }
                for prim in self.prims.values()
            ]
        }
        with open(output_path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)
