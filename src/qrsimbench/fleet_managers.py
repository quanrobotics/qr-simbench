"""External fleet managers, found by name.

A scenario's ``fleet_manager`` is ``"qr"`` (the built-in mission commanders, optionally with a fleet layer
plugin) or the name of an external fleet manager. External managers are plugin modules registered under the
``qrsimbench.fleet_managers`` entry-point group (or an optional ``qrsimbench.baselines.launch`` module).
A plugin module provides:

    MANAGERS: set[str]          names it handles
    LABELS: dict[str, str]      display names (reports, replay)
    WAIT_FOR_NAV2: set[str]     managers that need every robot's Nav2 up before they start
    BACKGROUND: set[str]        process names that run for the whole shift (the runner does not wait on them)
    def start(manager, resolved, run_dir, logs) -> list[Child]
"""

from __future__ import annotations

import importlib
from functools import lru_cache
from types import ModuleType


@lru_cache(maxsize=1)
def plugins() -> tuple[ModuleType, ...]:
    mods: list[ModuleType] = []
    try:
        from importlib.metadata import entry_points

        for ep in entry_points(group="qrsimbench.fleet_managers"):
            try:
                mods.append(ep.load())
            except Exception:  # a broken plugin must not break runs that do not use it
                continue
    except Exception:
        pass
    try:
        mods.append(importlib.import_module("qrsimbench.baselines.launch"))
    except ModuleNotFoundError:
        pass
    return tuple(mods)


def find(name: str) -> ModuleType | None:
    for m in plugins():
        if name in getattr(m, "MANAGERS", ()):
            return m
    return None


def label(name: str) -> str | None:
    m = find(name)
    return getattr(m, "LABELS", {}).get(name) if m else None
