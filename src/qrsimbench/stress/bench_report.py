"""HTML report for a benchmark (configurations x repetitions, baseline comparison)."""

from __future__ import annotations

import io
import json
import math
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from jinja2 import Template  # noqa: E402

from qrsimbench.reporting.html import TEMPLATES  # noqa: E402

PAGE = Template("""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Benchmark · QR-SimBench</title>
<style>{{ css }} td.best{font-weight:700} .win{color:#15803d;font-weight:700} .lose{color:#b91c1c}</style></head>
<body><div class="wrap">
<header class="brand"><div><b>QR-SimBench</b> <span>Benchmark</span></div><span>generated {{ generated }}</span></header>
<div class="hero"><div><h1>{{ s.name }}</h1><div class="sub">{{ s.base }} · {{ s.reps }} repetitions per configuration · baseline <b>{{ s.baseline }}</b></div></div></div>
<section><h2>Configurations</h2><table><tr><th>Config</th><th>Changes from the base scenario</th><th style="text-align:right">Valid runs</th><th>Verdicts</th><th style="text-align:right">Infra failures</th></tr>
{% for c, ch in s.configs.items() %}<tr><td><b>{{ c }}</b></td><td><code>{{ ch or "base scenario (full QR stack)" }}</code></td><td class="num">{{ t[c].n }}</td>
<td>{% for k in ["PASS","WARN","FAIL"] %}<span class="verdict {{ k }}" style="font-size:12px;padding:1px 8px;margin-right:4px">{{ t[c].verdicts[k] }}</span>{% endfor %}</td><td class="num">{{ t[c].infra_failures }}</td></tr>{% endfor %}</table></section>
<section><h2>Results: mean [95 % bootstrap CI], p vs baseline (permutation test)</h2><table><tr><th>Metric</th>{% for c in s.configs %}<th style="text-align:right">{{ c }}</th>{% endfor %}</tr>
{% for k, (label, higher) in s.metrics.items() %}<tr><td>{{ label }} <span class="u">{{ "↑" if higher else "↓" }}</span></td>
{% for c in s.configs %}{% set m = t[c].metrics[k] %}<td class="num">{% if m.mean is not none %}{{ "%.2f"|format(m.mean) }}<br><span class="u" style="font-size:11px">[{{ fmtci(m.ci) }}]{% if m.p_vs_baseline is not none %} p={{ "%.3f"|format(m.p_vs_baseline) }}{% endif %}</span>
{% if m.beats_baseline %}<br><span class="win">beats baseline</span>{% elif m.p_vs_baseline is not none and m.p_vs_baseline < 0.05 %}<br><span class="lose">worse</span>{% endif %}{% else %}—{% endif %}</td>{% endfor %}</tr>{% endfor %}</table>
<p class="u" style="font-size:12px">"Beats baseline" requires the better direction and p &lt; 0.05. Contact = centre distance below 0.45 m (AMR and person/robot bodies touching). Simulation results; not a safety certification and not a prediction of real-world performance.</p></section>
<section><h2>Distributions</h2>{{ plots|safe }}</section>
<footer>Every number traces back to replayable runs listed in results.json. Upstream: Gazebo Harmonic, Nav2, ROS 2 Jazzy.</footer>
</div></body></html>""")


def _fmtci(ci):
    lo, hi = ci
    return "—" if any(map(lambda v: v is None or (isinstance(v, float) and math.isnan(v)), ci)) else f"{lo:.2f}–{hi:.2f}"


def render_bench_report(bdir: Path) -> Path:
    s = json.loads((bdir / "summary.json").read_text())
    t = s["table"]
    keys = list(s["metrics"])
    cols = 4
    rows = math.ceil(len(keys) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(14, 3 * rows))
    for ax, k in zip(axes.flat, keys, strict=False):
        data = [t[c]["metrics"][k]["values"] or [math.nan] for c in s["configs"]]
        ax.boxplot(data, tick_labels=[c.split("_")[0] for c in s["configs"]], widths=0.5)
        ax.set_title(s["metrics"][k][0], fontsize=9)
    for ax in list(axes.flat)[len(keys):]:
        ax.axis("off")
    buf = io.StringIO()
    fig.tight_layout()
    fig.savefig(buf, format="svg", transparent=True)
    plt.close(fig)
    svg = buf.getvalue()[buf.getvalue().find("<svg"):]
    html = PAGE.render(css=(TEMPLATES / "base.css").read_text(), s=s, t=t, plots=svg, fmtci=_fmtci,
                       generated=datetime.now().strftime("%Y-%m-%d %H:%M"))
    out = bdir / "bench_report.html"
    out.write_text(html)
    return out
