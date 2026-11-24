"""qrsimbench command-line interface (spec §14)."""

from __future__ import annotations

from pathlib import Path

import typer

from qrsimbench import SCHEMA_VERSION, __version__
from qrsimbench.config.loader import ScenarioError, load_scenario, resolve

app = typer.Typer(
    add_completion=False,
    no_args_is_help=True,
    help="QR-SimBench: Scenario-to-Evidence Validation for Physical AI.",
)


def _not_yet(issue: str) -> None:
    typer.secho(f"not implemented yet ({issue})", fg=typer.colors.YELLOW, err=True)
    raise typer.Exit(code=2)


@app.callback(invoke_without_command=True)
def main(
    version: bool = typer.Option(False, "--version", help="Print version and exit."),
) -> None:
    if version:
        typer.echo(f"qrsimbench {__version__} (schema {SCHEMA_VERSION})")
        raise typer.Exit()


@app.command()
def doctor() -> None:
    """Check that ROS 2, Gazebo, Nav2, rosbag2/MCAP and assets are ready."""
    from qrsimbench.doctor import run_doctor

    raise typer.Exit(code=run_doctor())


@app.command()
def validate(scenario: Path, resolve_profiles: bool = typer.Option(True, "--resolve/--no-resolve")) -> None:
    """Validate a scenario YAML file (and resolve its profiles)."""
    try:
        sc = load_scenario(scenario)
        if resolve_profiles:
            resolve(sc)
    except ScenarioError as e:
        typer.secho(f"[FAIL] {e}", fg=typer.colors.RED, err=True)
        raise typer.Exit(code=1) from None
    typer.secho(
        f"[PASS] {scenario}: '{sc.experiment.name}' valid (schema {sc.schema_version}, "
        f"{len(sc.events)} event(s))",
        fg=typer.colors.GREEN,
    )


@app.command()
def run(scenario: Path, headless: bool = typer.Option(False, "--headless", help="No Gazebo GUI.")) -> None:
    """Run a scenario end to end and write an evidence bundle."""
    import sys

    from qrsimbench.runner.experiment import run_experiment

    try:
        _, verdict = run_experiment(scenario, ["qrsimbench", *sys.argv[1:]], headless=headless)
    except (ScenarioError, RuntimeError) as e:
        typer.secho(f"[FAIL] {e}", fg=typer.colors.RED, err=True)
        raise typer.Exit(code=1) from None
    raise typer.Exit(code=0 if verdict in ("PASS", "WARN") else 3)


@app.command()
def sweep(
    scenario: Path,
    reps: int = typer.Option(3, "--reps", min=1, help="Number of repetitions."),
    headless: bool = typer.Option(True, "--headless/--gui"),
) -> None:
    """Run a scenario REPS times and write aggregate statistics (sweep_*.json in runs/)."""
    import json
    import sys
    from datetime import datetime

    from qrsimbench.recording.manifest import RUNS_DIR
    from qrsimbench.reporting.aggregate import aggregate, format_table
    from qrsimbench.runner.experiment import run_experiment

    dirs = []
    for i in range(reps):
        typer.echo(f"--- repetition {i + 1}/{reps}")
        d, _ = run_experiment(scenario, ["qrsimbench", *sys.argv[1:]], headless=headless)
        dirs.append(d)
    agg = aggregate(dirs)
    out = RUNS_DIR / f"sweep_{datetime.now().strftime('%Y-%m-%dT%H%M%S')}_{scenario.stem}.json"
    out.write_text(json.dumps(agg, indent=2))
    typer.echo(format_table(agg))
    typer.echo(str(out))


@app.command()
def stats(
    pattern: str = typer.Argument(..., help="Scenario name, e.g. baseline"),
    all_configs: bool = typer.Option(False, "--all", help="Include runs with other resolved configs."),
) -> None:
    """Aggregate statistics over recorded runs of a scenario (same resolved config as the latest run)."""
    import json

    from qrsimbench.recording.manifest import RUNS_DIR
    from qrsimbench.reporting.aggregate import aggregate, format_table

    dirs = sorted(d for d in RUNS_DIR.glob(f"*_{pattern}*") if (d / "manifest.json").exists()
                  and json.loads((d / "manifest.json").read_text()).get("scenario_name") == pattern)
    if not dirs:
        typer.secho(f"no runs for scenario '{pattern}'", fg=typer.colors.RED, err=True)
        raise typer.Exit(code=1)
    if not all_configs:
        sha = lambda d: json.loads((d / "manifest.json").read_text()).get("resolved_sha256")  # noqa: E731
        latest = sha(dirs[-1])
        dirs = [d for d in dirs if sha(d) == latest]
        typer.echo(f"config {latest[:12]} (use --all to include other configs)")
    typer.echo(format_table(aggregate(dirs)))


@app.command()
def report(run_id: str) -> None:
    """(Re)generate the HTML evidence report for a run (id, path or latest:<scenario>)."""
    from qrsimbench.reporting.compare import resolve_run
    from qrsimbench.reporting.html import render_report

    typer.echo(render_report(resolve_run(run_id)))


@app.command()
def compare(run_a: str, run_b: str) -> None:
    """Compare two runs (ids, paths or latest:<scenario>) and write a delta report."""
    from qrsimbench.reporting.compare import render_compare

    typer.echo(render_compare(run_a, run_b))


@app.command("list")
def list_runs() -> None:
    """List recorded runs."""
    import json

    from qrsimbench.recording.manifest import RUNS_DIR

    for d in sorted(p for p in RUNS_DIR.iterdir() if (p / "manifest.json").exists()):
        m = json.loads((d / "manifest.json").read_text())
        typer.echo(f"{d.name:50s} {m.get('verdict') or '-':5s} {m.get('terminal_result') or '-'}")


@app.command()
def reproduce(run_id: str, headless: bool = typer.Option(False, "--headless")) -> None:
    """Re-run a recorded run from its frozen source scenario."""
    from qrsimbench.reporting.compare import resolve_run

    d = resolve_run(run_id)
    typer.echo(f"reproducing {d.name} (source scenario frozen in the run folder)")
    run(d / "scenario.source.yaml", headless=headless)


@app.command()
def stress(spec: Path, limit: int = typer.Option(None, "--limit", help="Run only the first N variants."),
           gui: bool = typer.Option(False, "--gui")) -> None:
    """Run a randomized stress campaign and mine failure modes (writes runs/stress_*/)."""
    from qrsimbench.stress.engine import run_stress

    out = run_stress(spec, headless=not gui, limit=limit)
    typer.echo(f"{out / 'stress_report.html'}")


@app.command()
def bench(spec: Path, gui: bool = typer.Option(False, "--gui")) -> None:
    """Benchmark configurations of a scenario against a baseline (repeated runs + statistics)."""
    from qrsimbench.stress.bench import run_bench

    typer.echo(run_bench(spec, headless=not gui) / "bench_report.html")


@app.command()
def rescore(bench_dirs: list[Path]) -> None:
    """Re-analyze every run of the given benches (after an analysis fix) and rebuild their reports."""
    from qrsimbench.stress.bench import rescore_bench

    for b in bench_dirs:
        typer.echo(rescore_bench(b) / "bench_report.html")


@app.command("bench-merge")
def bench_merge(out: Path, bench_dirs: list[Path], baseline: str = typer.Option(..., "--baseline")) -> None:
    """Merge benches of one scenario (e.g. frozen baselines + a later QR version) into one table."""
    from qrsimbench.stress.bench import merge_benches

    typer.echo(merge_benches(out, bench_dirs, baseline) / "bench_report.html")


@app.command()
def console(port: int = typer.Option(8765, "--port")) -> None:
    """Start the QR-OS Console (local web UI) at http://127.0.0.1:PORT."""
    from qrsimbench.console.app import serve

    typer.echo(f"QR-OS Console on http://127.0.0.1:{port}  (Ctrl+C to stop)")
    serve(port)


data_app = typer.Typer(no_args_is_help=True, help="QR-RobotData: episode catalogue built from runs.")
app.add_typer(data_app, name="data")


@data_app.command("ingest")
def data_ingest(run_id: str = typer.Argument(None, help="Run id/path; omit to ingest all runs."),
                force: bool = typer.Option(False, "--force")) -> None:
    """Convert run bundles into indexed episodes."""
    from qrsimbench.recording.manifest import RUNS_DIR
    from qrsimbench.reporting.compare import resolve_run
    from qrsimbench.robotdata.store import Catalogue, have_parquet

    cat = Catalogue()
    if run_id:
        meta = cat.ingest(resolve_run(run_id), force=force)
        typer.echo(meta.episode_id if meta else "already in catalogue (use --force)")
    else:
        added, errors = cat.ingest_all(RUNS_DIR, force=force)
        typer.echo(f"added {added} episode(s); {len(errors)} run(s) skipped")
        for e in errors[:10]:
            typer.echo(f"  skipped {e}")
    if not have_parquet():
        typer.secho("note: pyarrow not installed, tables stored as .npz", fg=typer.colors.YELLOW)


@data_app.command("list")
def data_list(scenario: str = typer.Option(None), verdict: str = typer.Option(None),
              summary: bool = typer.Option(False, "--summary")) -> None:
    """List episodes in the catalogue (or a per-scenario summary)."""
    from qrsimbench.robotdata.store import Catalogue

    cat = Catalogue()
    if summary:
        for r in cat.summary():
            typer.echo(f"{r['scenario']:20s} episodes {r['n']:4d}  success {r['n_success']:4d}  "
                       f"{r['minutes']:6.1f} min  {r['samples']:7d} samples")
        return
    for r in cat.query(scenario, verdict):
        typer.echo(f"{r['episode_id']:52s} {r['robot_type']:15s} {r['verdict']:5s} {r['duration_s']:6.1f}s")


@data_app.command("export")
def data_export(out: Path = typer.Argument(..., help="Output dataset directory"),
                scenario: str = typer.Option(None), fmt: str = typer.Option("lerobot", "--format")) -> None:
    """Export episodes as a dataset (LeRobot-style layout)."""
    from qrsimbench.robotdata.export import export_lerobot
    from qrsimbench.robotdata.store import Catalogue

    if fmt != "lerobot":
        raise typer.BadParameter("only --format lerobot is supported")
    typer.echo(export_lerobot(Catalogue(), out, scenario=scenario))


if __name__ == "__main__":
    app()
