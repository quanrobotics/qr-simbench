# Contributing to QR-SimBench

Thank you for helping. Bug reports, new scenarios, world importers, metrics and documentation fixes are
all welcome.

## Before you start

- For anything larger than a small fix, open an issue first so we can agree on the approach.
- Be kind and constructive: see [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).
- Security problems: do **not** open a public issue, see [SECURITY.md](SECURITY.md).

## Development setup

Analysis, reporting and the tests run without ROS:

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e '.[dev]'
ruff check src tests scripts
pytest -q
```

Running simulations needs the ROS environment from the README (`environment.ros.yml`).

## Adding things

- **A scenario:** add a YAML file under `examples/`, run `qrsimbench validate <file>`, then run it at least
  three times and put the verdicts in the pull request.
- **A world:** write a layout file under `simulation/worlds/` and build it with `scripts/build_world.py`;
  for an existing map, start from `scripts/importers/`.
- **A metric:** implement it in `src/qrsimbench/analysis/`, document it in `docs/metrics.md`, add a unit test
  with a fixture.
- **A fleet coordination layer:** publish it as a separate package registered under the
  `qrsimbench.fleet_layers` entry point (see `src/qrsimbench/navigation/mission.py`).

## Pull requests

- Keep each pull request to one change, with tests.
- `ruff check` and `pytest -q` must pass (CI runs both).
- Say where any copied file comes from and under which licence; we can only accept Apache-2.0-compatible
  material.

## Sign-off (DCO)

Every commit must be signed off, certifying the [Developer Certificate of Origin](https://developercertificate.org/):

```bash
git commit -s -m "Describe the change"
```

Contributions are accepted under the Apache-2.0 licence of this repository.
