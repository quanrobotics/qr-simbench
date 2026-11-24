"""Stable hashing for provenance."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: str | Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_obj(obj: Any) -> str:
    """Hash a JSON-able object independent of key order."""
    return sha256_bytes(json.dumps(obj, sort_keys=True, separators=(",", ":")).encode())


def sha256_tree(path: str | Path) -> str:
    """Hash a file or every file under a directory (path-sorted)."""
    p = Path(path)
    if p.is_file():
        return sha256_file(p)
    h = hashlib.sha256()
    for f in sorted(q for q in p.rglob("*") if q.is_file()):
        h.update(str(f.relative_to(p)).encode())
        h.update(sha256_file(f).encode())
    return h.hexdigest()
