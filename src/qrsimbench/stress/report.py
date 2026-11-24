"""Standalone HTML report for a stress campaign: outcome mix, failure modes, sensitivity, distributions."""

from __future__ import annotations

import io
import json
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from jinja2 import Template  # noqa: E402

from qrsimbench.reporting.html import TEMPLATES, fmt  # noqa: E402

COLORS = {"PASS": "#15803d", "WARN": "#b45309", "FAIL": "#b91c1c"}
KEYS = [("time_to_goal", "Time to goal [s]"), ("path_length", "Path length [m]"),
        ("min_clearance", "Min clearance [m]"), ("stopped_time", "Stopped time [s]")]

PAGE = Template("""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Stress campaign · QR-SimBench</title>
<style>{{ css }}</style></head><body><div class="wrap">
<header class="brand"><div><b>QR-SimBench</b> <span>Stress campaign</span></div><span>generated {{ generated }}</span></header>
<div class="hero"><div><h1>{{ s.name or "stress" }}</h1><div class="sub">Base scenario: {{ s.base }} · {{ s.n }} randomized variants</div></div></div>
<div class="cards">
{% for k in ["PASS","WARN","FAIL"] %}<div class="card"><div class="k">{{ k }}</div><div class="v"><span class="{{ k }}" style="padding:0 8px;border-radius:6px">{{ s.verdicts[k] }}</span></div></div>{% endfor %}
<div class="card"><div class="k">Failure modes</div><div class="v">{{ s.modes|length }}</div></div>
<div class="card"><div class="k">Parameters varied</div><div class="v">{{ s.params|length }}</div></div>
</div>
<section><h2>Failure modes (ranked)</h2>{% if s.modes %}<table><tr><th>Mode</th><th style="text-align:right">Runs</th><th>Parameter ranges where it occurred</th><th>Replay</th></tr>
{% for m in s.modes %}<tr><td><b>{{ m.mode }}</b></td><td class="num">{{ m.count }}</td><td>{% for p, r in m.param_ranges.items() %}<code>{{ p.split('.')[-1] }}</code> {{ "%.2f"|format(r[0]) }}–{{ "%.2f"|format(r[1]) }}<br>{% endfor %}</td>
<td><code>qrsimbench run {{ m.example_variant }}</code><br><span class="u">evidence: runs/{{ m.example }}</span></td></tr>{% endfor %}</table>
{% else %}<p>No failures found in this campaign.</p>{% endif %}</section>
<div class="grid2"><section><h2>Outcome vs parameters</h2>{{ scatter|safe }}</section>
<section><h2>Sensitivity (correlation with non-PASS)</h2><table><tr><th>Parameter</th><th style="text-align:right">r</th></tr>
{% for p, r in s.sensitivity.items() %}<tr><td><code>{{ p }}</code></td><td class="num">{{ "%+.2f"|format(r) }}</td></tr>{% endfor %}</table>
<p class="u" style="font-size:12px">Point-biserial correlation over {{ s.n }} runs; an indicator for where to look, not a causal claim.</p></section></div>
<section><h2>Metric distributions</h2>{{ dists|safe }}</section>
<section><h2>All variants</h2><table><tr><th>#</th><th>Verdict</th><th>Codes</th>{% for p in s.params %}<th>{{ p.split('.')[-1] }}</th>{% endfor %}<th style="text-align:right">Time [s]</th><th style="text-align:right">Min clear. [m]</th><th>Run</th></tr>
{% for r in results %}<tr><td>{{ loop.index0 }}</td><td><span class="verdict {{ r.verdict }}" style="font-size:12px;padding:1px 8px">{{ r.verdict }}</span></td><td>{{ r.codes|join(", ") }}</td>
{% for p in s.params %}<td class="num">{{ r.params[p] }}</td>{% endfor %}<td class="num">{{ fmt(r.metrics.time_to_goal, 1) }}</td><td class="num">{{ fmt(r.metrics.min_clearance) }}</td><td><code>{{ r.run_id }}</code></td></tr>{% endfor %}</table></section>
<footer>Engineering validation in simulation; not a safety certification and not a prediction of real-world performance.
Every variant is a complete scenario file; every run is a replayable evidence bundle.</footer></div></body></html>""")


def _svg(fig) -> str:
    buf = io.StringIO()
    fig.savefig(buf, format="svg", bbox_inches="tight", transparent=True)
    plt.close(fig)
    s = buf.getvalue()
    return s[s.find("<svg"):]


def render_stress_report(sdir: Path) -> Path:
    s = json.loads((sdir / "summary.json").read_text())
    results = json.loads((sdir / "results.json").read_text())
    numeric = [p for p in s["params"] if all(isinstance(r["params"][p], (int, float)) for r in results)]
    scatter = "<p>No numeric parameters.</p>"
    if len(numeric) >= 1:
        px, py = numeric[0], (numeric[1] if len(numeric) > 1 else None)
        fig, ax = plt.subplots(figsize=(6, 4))
        for v, c in COLORS.items():
            pts = [r for r in results if r["verdict"] == v]
            ax.scatter([r["params"][px] for r in pts],
                       [r["params"][py] for r in pts] if py else [0] * len(pts), c=c, label=v, s=40, alpha=0.85)
        ax.set_xlabel(px.split(".")[-1])
        ax.set_ylabel(py.split(".")[-1] if py else "")
        ax.legend(frameon=False)
        scatter = _svg(fig)
    fig, axes = plt.subplots(1, len(KEYS), figsize=(12, 2.8))
    for ax, (k, label) in zip(axes, KEYS, strict=True):
        data = [[r["metrics"].get(k) for r in results if r["verdict"] == v and r["metrics"].get(k) is not None]
                for v in COLORS]
        ax.boxplot([d if d else [np.nan] for d in data], tick_labels=list(COLORS), widths=0.5)
        ax.set_title(label, fontsize=10)
    html = PAGE.render(css=(TEMPLATES / "base.css").read_text(), s=s, results=results, fmt=fmt,
                       scatter=scatter, dists=_svg(fig), generated=datetime.now().strftime("%Y-%m-%d %H:%M"))
    out = sdir / "stress_report.html"
    out.write_text(html)
    return out
