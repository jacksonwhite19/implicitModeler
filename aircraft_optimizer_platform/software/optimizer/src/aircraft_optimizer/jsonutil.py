from __future__ import annotations

import json
from typing import Any


def dumps(value: Any) -> str:
    return json.dumps(value, indent=None, sort_keys=True, separators=(",", ":"))
