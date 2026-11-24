"""One line per run: verdict, codes and key metrics. Usage: python scripts/summarize_runs.py [glob]"""
import json
import sys
from pathlib import Path

pat = sys.argv[1] if len(sys.argv) > 1 else "*"
keys = ["time_to_goal", "path_length", "path_efficiency", "min_clearance", "goal_error", "stopped_time"]
print(f"{'run':40s} {'verdict':7s} " + " ".join(f"{k[:9]:>9s}" for k in keys) + "  codes / events")
for d in sorted(Path("runs").glob(pat)):
    f = d / "metrics.json"
    if not f.exists():
        continue
    m = json.loads(f.read_text())
    x = m.get("metrics", {})
    vals = " ".join(f"{x[k]:9.2f}" if isinstance(x.get(k), (int, float)) else f"{'-':>9s}" for k in keys)
    ev = json.loads((d / "events.json").read_text()) if (d / "events.json").exists() else []
    evs = ",".join(f"{e['id']}@{e.get('elapsed', 0):.0f}s:{'ok' if e.get('ok') else 'X'}"
                   for e in ev if not e["id"].startswith("_"))
    print(f"{d.name[:40]:40s} {m.get('verdict', '?'):7s} {vals}  {','.join(m.get('codes', []))} {evs}")
