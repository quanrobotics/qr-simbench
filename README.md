# QR-SimBench

**Scenario-to-evidence validation for mobile robots.** Describe a robot test once in YAML, run it in
simulation with one command, inject a test condition, record the robot's data, compute metrics, get a
PASS / WARN / FAIL engineering verdict, and receive a standalone HTML evidence report that you can compare
and reproduce.

```
YAML scenario → Gazebo + Nav2 run → injected event → MCAP recording → metrics → PASS/WARN/FAIL → HTML report → compare → reproduce
```

QR-SimBench is built and maintained by [QR](https://www.quanrobotics.org). It powers
[QR-Sim](https://qr-sim.quanrobotics.org), our hosted product.

> Verdicts are engineering validation results in simulation. They are **not** a safety certification and
> do not claim sim-to-real equivalence.

## What is QR's and what is upstream

QR-SimBench orchestrates standard open robotics software and does not modify it: ROS 2 Jazzy, Gazebo
Harmonic, Nav2, rosbag2 and MCAP (see [THIRD_PARTY.md](THIRD_PARTY.md)). QR's contribution is the scenario
contract, orchestration, event injection, provenance, metrics, verdicts, reports, comparison,
reproduction, stress campaigns, world builder and importers, and the robot-data export.

## Install (Ubuntu 22.04 or 24.04, no sudo, NVIDIA GPU recommended)

```bash
git clone https://github.com/quanrobotics/qr-simbench && cd qr-simbench
mamba env create -p ./env -f environment.ros.yml   # ROS 2 Jazzy + Gazebo Harmonic + Nav2 via RoboStack (~10 GB)
source activate.sh
pip install --no-deps -e .
qrsimbench doctor
```

Only the analysis and reporting side is needed to read existing runs: `pip install -e '.[dev]'` in any
Python 3.10+ environment (no ROS).

## Quick start

```bash
source activate.sh
qrsimbench validate examples/warehouse_amr/scenarios/blocked_aisle.yaml
qrsimbench run examples/warehouse_amr/scenarios/baseline.yaml --headless
qrsimbench run examples/warehouse_amr/scenarios/blocked_aisle.yaml --headless
qrsimbench compare latest:baseline latest:blocked_aisle
qrsimbench reproduce latest:blocked_aisle
qrsimbench sweep examples/warehouse_amr/scenarios/baseline.yaml --reps 5      # repeat + statistics
qrsimbench stress examples/warehouse_amr/stress/blocked_aisle_stress.yaml     # randomized variants
```

## See it

- **Live:** leave out `--headless` and Gazebo opens its 3D window while the robot drives.
- **Evidence report:** every run writes `report.html` (verdict, gates, metrics, path and clearance plots).
- **3D replay in the browser:** `qrsimbench replay latest:blocked_aisle` opens an interactive replay of the
  recorded run (robots, people, spawned obstacles, stations, timeline, speed controls). Several runs can be
  replayed together: `qrsimbench replay <run> <run> ...`.

| Scenario | Condition | Expected |
|---|---|---|
| `baseline` | clear route | PASS |
| `blocked_aisle` | a pallet appears in the aisle mid-mission | PASS (slower) |
| `narrow_aisle` | tight passage | PASS or WARN (low clearance) |
| `impossible_route` | goal cannot be reached | clean FAIL |
| `fleet_shift` | three robots, shared aisles, stock Nav2 per robot | depends on the run |

## Run bundle

Each run writes `runs/<timestamp>_<scenario>/`: `manifest.json` (hashes, versions, hardware, command,
result), `scenario.source.yaml`, `scenario.resolved.yaml`, `nav2_params.yaml`, `environment.json`,
`events.json`, `mission.json`, `metrics.json`, `bag/*.mcap`, `report.html`, `plots/`, `logs/` and
`reproduce.sh`. Metric definitions: [docs/metrics.md](docs/metrics.md).

## Worlds and importers

Worlds are generated from a layout file (`scripts/build_world.py`); robots from a profile
(`scripts/build_robot.py`). Importers turn an existing ROS map (`scripts/importers/ros_map.py`) or the AWS
RoboMaker small warehouse (`scripts/importers/aws_small_warehouse.py`) into a layout.

## Robot data

Every run can be indexed into a local catalogue and exported in the LeRobot dataset layout:

```bash
qrsimbench data ingest && qrsimbench data export out/ds --format lerobot
```

## Fleet coordination plugins

Multi-robot scenarios run stock Nav2 on every robot. A fleet coordination layer can be added as a plugin
registered under the `qrsimbench.fleet_layers` entry point (see `src/qrsimbench/navigation/mission.py`).
QR's own fleet layer is not part of this repository.

## Contributing, security, conduct

See [CONTRIBUTING.md](CONTRIBUTING.md), [SECURITY.md](SECURITY.md) and
[CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md). Questions: GitHub Discussions.

## Citing

See [CITATION.cff](CITATION.cff) (GitHub shows a "Cite this repository" button).

## Licence

Apache-2.0 ([LICENSE](LICENSE), [NOTICE](NOTICE)). The QR name and logo are not covered by the licence
([TRADEMARKS.md](TRADEMARKS.md)). QR's fleet coordination technology is patent pending and is not
included in this repository.
