"""Engineering Validation Verdict: PASS / WARN / FAIL (spec §11 layer 6, §18).

Not a safety certification.
"""

from __future__ import annotations

from typing import Any

RESULT_CODES = {
    "SUCCEEDED": "MISSION_SUCCESS",
    "FAILED": "GOAL_FAILED",
    "CANCELED": "GOAL_FAILED",
    "TIMEOUT": "TIMEOUT",
    "NO_PATH": "NO_PATH",
}


def evaluate(metrics: dict[str, Any], gates: dict[str, dict[str, Any]], prolonged_stop_s: float = 20.0):
    """Return (verdict, codes, gate_results)."""
    codes: list[str] = [RESULT_CODES.get(metrics.get("nav2_result", ""), "GOAL_FAILED")]
    gate_results: list[dict[str, Any]] = []
    verdict = "PASS"

    def record(name, ok, detail, severity="FAIL"):
        nonlocal verdict
        gate_results.append({"gate": name, "passed": ok, "detail": detail, "severity": severity})
        if not ok:
            if severity == "FAIL":
                verdict = "FAIL"
            elif verdict == "PASS":
                verdict = "WARN"

    if not metrics.get("success"):
        record("success", False, f"Nav2 result {metrics.get('nav2_result')}")

    for name, g in gates.items():
        if name == "success" and g.get("equals") is not None:
            if metrics.get("success") == g["equals"]:
                record(name, True, f"success == {g['equals']}")
        elif name == "min_clearance":
            mc = metrics.get("min_clearance")
            if mc is None:
                record(name, False, "no clearance samples while moving", severity="WARN")
                continue
            if g.get("greater_than_m") is not None and mc <= g["greater_than_m"]:
                record(name, False, f"min clearance {mc:.2f} m <= {g['greater_than_m']} m")
                codes.append("LOW_CLEARANCE_WARNING")
            elif g.get("warn_below_m") is not None and mc < g["warn_below_m"]:
                record(name, False, f"min clearance {mc:.2f} m < {g['warn_below_m']} m", severity="WARN")
                codes.append("LOW_CLEARANCE_WARNING")
            else:
                record(name, True, f"min clearance {mc:.2f} m")
        elif name == "timeout" and g.get("less_than_seconds") is not None:
            t = metrics.get("time_to_goal")
            if t is not None:
                record(name, t < g["less_than_seconds"], f"time to goal {t:.1f} s")

    st = metrics.get("stopped_time")
    if st is not None and st > prolonged_stop_s:
        codes.append("PROLONGED_STOP")
        record("prolonged_stop", False, f"stopped {st:.1f} s", severity="WARN")

    return verdict, codes, gate_results
