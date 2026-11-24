"""One-command experiment lifecycle (QRB-006/007/008), spec §11 layer 3."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from qrsimbench import fleet_managers
from qrsimbench.analysis.bag_reader import GT_TOPIC, fleet_ids, read_run
from qrsimbench.analysis.metrics import compute_metrics
from qrsimbench.analysis.verdict import evaluate
from qrsimbench.config.loader import REPO_ROOT, load_scenario, resolve
from qrsimbench.recording import manifest as mf
from qrsimbench.runner.process import Child, group_rss_mb, mem_available_mb

MIN_FREE_MB = 1200  # stop a run (as an infrastructure failure) before the kernel OOM killer acts


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def deep_merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in over.items():
        out[k] = deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def _has(module: str) -> bool:
    """True if an optional qrsimbench module (fleet layer, baselines) is part of this build."""
    import importlib.util

    try:
        return importlib.util.find_spec(f"qrsimbench.{module}") is not None
    except ModuleNotFoundError:
        return False


def write_nav2_params(resolved: dict, run_dir: Path) -> Path:
    """Freeze the exact Nav2 parameters used into the run folder (upstream defaults + profile overrides)."""
    import yaml

    prefix = Path(os.environ.get("CONDA_PREFIX", "/opt/ros/jazzy"))
    upstream = prefix / "share/nav2_bringup/params/nav2_params.yaml"
    params = yaml.safe_load(upstream.read_text())
    over = resolved.get("resolved_profiles", {}).get("nav2", {}).get("overrides") or {}
    over = deep_merge(over, resolved.get("navigation", {}).get("overrides") or {})
    if resolved.get("fleet"):
        aware = (resolved.get("fleet_policy") or {}).get("awareness", True)
        over = deep_merge(over, FLEET_PARAM_OVERRIDES if aware else FLEET_TOPIC_OVERRIDES)
        if aware:
            from qrsimbench.navigation.mission import load_fleet_layer

            layer = load_fleet_layer()  # optional plugin: extra Nav2 parameters for its features
            if layer is not None and hasattr(layer, "nav2_overrides"):
                over = deep_merge(over, layer.nav2_overrides(resolved))
    out = run_dir / "nav2_params.yaml"
    class NoAliasDumper(yaml.SafeDumper):  # ROS 2 param parser rejects YAML anchors/aliases
        def ignore_aliases(self, data):
            return True

    out.write_text(yaml.dump(deep_merge(params, over), Dumper=NoAliasDumper, sort_keys=False))
    return out


def demo_variant(rel: str) -> str:
    """simulation/x/name.sdf.xacro -> simulation/x/name_demo.sdf.xacro (adds frame-saving cameras)."""
    return rel.replace(".sdf.xacro", "_demo.sdf.xacro")


# Fleet runs: sensor topics must be namespaced ('<robot_namespace>' is substituted by Nav2's bringup),
# and each robot's costmaps also consume the other robots' shared positions (fleet_awareness.py).
_FLEET_SOURCE = {"topic": "<robot_namespace>/fleet_obstacles", "data_type": "PointCloud2", "marking": True,
                 "clearing": False, "max_obstacle_height": 2.0, "min_obstacle_height": 0.0,
                 # ranges are measured from the cloud frame origin (map origin), so keep them large
                 "obstacle_max_range": 100.0, "obstacle_min_range": 0.0, "raytrace_max_range": 100.0}
FLEET_PARAM_OVERRIDES = {
    "local_costmap": {"local_costmap": {"ros__parameters": {"voxel_layer": {
        "observation_sources": "scan fleet", "scan": {"topic": "<robot_namespace>/scan"},
        "fleet": _FLEET_SOURCE}}}},
    "global_costmap": {"global_costmap": {"ros__parameters": {"obstacle_layer": {
        "observation_sources": "scan fleet", "scan": {"topic": "<robot_namespace>/scan"},
        "fleet": _FLEET_SOURCE}}}},
    "collision_monitor": {"ros__parameters": {"scan": {"topic": "<robot_namespace>/scan"}}},
}
# Stock multi-robot behaviour: only the namespaced sensor topics, no shared positions.
_NS_SCAN = {"scan": {"topic": "<robot_namespace>/scan"}}
FLEET_TOPIC_OVERRIDES = {
    "local_costmap": {"local_costmap": {"ros__parameters": {"voxel_layer": _NS_SCAN}}},
    "global_costmap": {"global_costmap": {"ros__parameters": {"obstacle_layer": _NS_SCAN}}},
    "collision_monitor": {"ros__parameters": _NS_SCAN},
}
FLEET_TOPICS = ["odom", "scan", "cmd_vel", "plan", "amcl_pose", "tf", "tf_static"]


def fleet_command(resolved: dict, params_file: Path, record: bool) -> list[str]:
    world = resolved["resolved_profiles"]["world"]
    robot = resolved["resolved_profiles"]["robot"]
    w = demo_variant(world["sim_world"]) if record else world["sim_world"]
    r = demo_variant(robot["sim_robot_sdf"]) if record else robot["sim_robot_sdf"]
    robots = "; ".join(f"{m['id']}={{x: {m['start_pose']['x']}, y: {m['start_pose']['y']}, "
                       f"yaw: {m['start_pose'].get('yaw', 0.0)}}}" for m in resolved["fleet"])
    return ["ros2", "launch", str(REPO_ROOT / "simulation/launch/qr_fleet.launch.py"),
            f"world:={REPO_ROOT / w}", f"map:={REPO_ROOT / world['map']}", f"params_file:={params_file}",
            f"robot_sdf:={REPO_ROOT / r}", f"robots:={robots}"]


def sim_command(resolved: dict, params_file: Path | None = None, record: bool = False) -> list[str]:
    if resolved.get("fleet"):
        return fleet_command(resolved, params_file, record)
    world = resolved.get("resolved_profiles", {}).get("world", {})
    # With a demo GUI config, the server always runs headless and our own GUI client attaches.
    headless = resolved["runtime"].get("headless", False) or bool(world.get("gui_configs"))
    sp = resolved["mission"]["start_pose"]
    cmd = ["ros2", "launch", "nav2_bringup", "tb3_simulation_launch.py",
           f"headless:={'True' if headless else 'False'}", "use_rviz:=False",
           f"x_pose:={sp['x']}", f"y_pose:={sp['y']}", f"yaw:={sp.get('yaw', 0.0)}"]
    if world.get("sim_world"):
        w = demo_variant(world["sim_world"]) if record else world["sim_world"]
        cmd.append(f"world:={REPO_ROOT / w}")
    if world.get("sim_world") and world.get("map"):
        cmd.append(f"map:={REPO_ROOT / world['map']}")
    if params_file:
        cmd.append(f"params_file:={params_file}")
    robot = resolved.get("resolved_profiles", {}).get("robot", {})
    if robot.get("sim_robot_sdf"):
        r = demo_variant(robot["sim_robot_sdf"]) if record else robot["sim_robot_sdf"]
        cmd.append(f"robot_sdf:={REPO_ROOT / r}")
    return cmd


INFRA_CODES = {"NAV_STACK_NOT_READY", "SIMULATOR_START_FAILURE", "RECORDING_FAILURE"}


def _fleet_settled(run_dir: Path, ids: list[str]) -> bool:
    """Every robot has either finished its mission or was broken down by a scenario event: the shift
    is decided, so there is no need to wait for a broken robot's mission timeout."""
    if not ids:
        return False
    ev = run_dir / "events_scenario.json"
    try:
        disabled = {e.get("robot") for e in (json.loads(ev.read_text()) if ev.exists() else [])
                    if e.get("type") == "disable_robot" and e.get("ok")}
        if not disabled:
            return False
        return all(i in disabled or json.loads((run_dir / f"mission_{i}.json").read_text()).get("mission_end")
                   for i in ids)
    except (OSError, ValueError):
        return False


def wait_nav2_ready(ids: list[str], timeout: float) -> bool:
    """Block until every fleet robot's navigate_to_pose action server is advertised."""
    deadline = time.time() + timeout
    want = {f"/{i}/navigate_to_pose" for i in ids}
    while time.time() < deadline:
        out = subprocess.run(["ros2", "action", "list"], capture_output=True, text=True, timeout=30).stdout
        missing = want - set(out.split())
        if not missing:
            log(f"Nav2 up on all {len(ids)} robots")
            return True
        time.sleep(5)
    log(f"Nav2 not up on: {sorted(missing)}")
    return False


def analyze(run_dir: Path, resolved: dict) -> dict:
    mf_path = run_dir / "mission.json"
    mission = json.loads(mf_path.read_text()) if mf_path.exists() else {}
    ids = fleet_ids(resolved)
    files = [run_dir / f"mission_{i}.json" for i in ids]
    not_ready = [f for f in files if f.exists() and not json.loads(f.read_text()).get("mission_start")]
    guard = run_dir / "infrastructure_failure.json"
    if guard.exists():  # stopped by the runner's memory guard
        out = {"verdict": "FAIL", "verdict_label": "Engineering Validation Verdict",
               "codes": ["MEMORY_GUARD"],
               "gates": [], "metrics": {"success": False, "nav2_result": "MEMORY_GUARD"},
               "infrastructure_failure": True, "guard": json.loads(guard.read_text())}
        (run_dir / "metrics.json").write_text(json.dumps(out, indent=2))
        return out
    if not mission.get("mission_start") or not_ready:  # a stack never sent a goal: infrastructure failure
        code = mission.get("result") or "NAV_STACK_NOT_READY"
        out = {"verdict": "FAIL", "verdict_label": "Engineering Validation Verdict", "codes": [code],
               "gates": [],
               "metrics": {"success": False, "nav2_result": code}, "infrastructure_failure": True}
        (run_dir / "metrics.json").write_text(json.dumps(out, indent=2))
        return out
    if resolved.get("fleet"):
        from qrsimbench.analysis.fleet import analyze_fleet

        out = analyze_fleet(run_dir, resolved)
        (run_dir / "metrics.json").write_text(json.dumps(out, indent=2))
        return out
    gates = resolved.get("gates", {})
    warn = gates.get("min_clearance", {}).get("warn_below_m") or 0.3
    run = read_run(run_dir, resolved)
    metrics = compute_metrics(run, clearance_threshold=warn)
    verdict, codes, gate_results = evaluate(metrics, gates)
    out = {"verdict": verdict, "verdict_label": "Engineering Validation Verdict", "codes": codes,
           "gates": gate_results, "metrics": metrics}
    (run_dir / "metrics.json").write_text(json.dumps(out, indent=2))
    return out


def run_experiment(scenario_path: Path, argv: list[str], headless: bool = False) -> tuple[Path, str]:
    if shutil.which("ros2") is None:
        raise RuntimeError("ros2 not on PATH: activate the ROS env first (see README)")
    sc = load_scenario(scenario_path)
    resolved = resolve(sc)
    if headless:
        resolved["runtime"]["headless"] = True
    run_dir = mf.create_run_dir(sc.experiment.name)
    logs = run_dir / "logs"
    logs.mkdir()
    manifest = mf.write_bundle_start(run_dir, scenario_path.resolve(), resolved, argv)
    mf.write_reproduce_script(run_dir)
    log(f"run {run_dir.name}")
    record = bool(os.environ.get("QRSB_RECORD_VIDEO"))

    sim = bag = gt = cams = aware = actors = safety = scen_events = None
    cmdrs: list[Child] = []
    guis: list[Child] = []
    result = "SIMULATOR_START_FAILURE"
    try:
        rt = resolved["runtime"]
        sim_env = {"QRSB_BRINGUP_STAGGER_S": str(rt.get("bringup_stagger_s", 6.0))}
        if rt.get("nav_launch") == "slim":
            sim_env["QRSB_NAV_BRINGUP"] = str(REPO_ROOT / "simulation/launch/qr_bringup_slim.launch.py")
        sim = Child("sim", sim_command(resolved, write_nav2_params(resolved, run_dir), record), logs,
                    env=sim_env)
        time.sleep(3)
        if not sim.alive:
            raise RuntimeError("simulator exited during startup, see logs/sim.log")
        world = resolved.get("resolved_profiles", {}).get("world", {})
        if not resolved["runtime"].get("headless") and world.get("gui_configs") and not record:
            share = Path(os.environ.get("CONDA_PREFIX", "/opt/ros/jazzy")) / "share"
            res = os.pathsep.join([os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                                   str(share / "nav2_minimal_tb3_sim" / "models"), str(share)])
            guis = [Child(f"gui{i}", ["gz", "sim", "-g", "--force-version", "8",
                                      "--gui-config", str(REPO_ROOT / cfg)], logs,
                          env={"GZ_SIM_RESOURCE_PATH": res})
                    for i, cfg in enumerate(world["gui_configs"])]
            follow = world.get("gui_follow")
            if follow:  # chase camera on the GUI that has the CameraTracking plugin
                time.sleep(6)
                model_name = resolved.get("resolved_profiles", {}).get("robot", {}).get(
                    "gz_model_name", "turtlebot3_waffle")
                off = follow.get("offset", [-1.2, -0.9, 4.2])
                for svc, rtype, req in (
                    ("/gui/follow", "gz.msgs.StringMsg", f'data: "{model_name}"'),
                    ("/gui/follow/offset", "gz.msgs.Vector3d", f"x: {off[0]} y: {off[1]} z: {off[2]}"),
                ):
                    subprocess.run(["gz", "service", "-s", svc, "--reqtype", rtype, "--reptype",
                                    "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                                   capture_output=True, timeout=10)
        if record:
            from qrsimbench.reporting.video import camera_specs

            cams = Child("cameras", [sys.executable, "-m", "qrsimbench.recording.camera_video",
                                     str(run_dir / "video"), *[c for c, _ in camera_specs(resolved)]], logs)
        ids = fleet_ids(resolved)
        robot = resolved.get("resolved_profiles", {}).get("robot", {})
        # (gz model name, ROS ground-truth topic) per robot
        gt_map = ([(i, f"/{i}{GT_TOPIC}") for i in ids] if ids
                  else [(robot.get("gz_model_name", "turtlebot3_waffle"), GT_TOPIC)])
        bridge = ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                  *[f"/model/{m}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose" for m, _ in gt_map],
                  "--ros-args", *[a for m, t in gt_map for a in ("-r", f"/model/{m}/pose:={t}")]]
        gt = Child("gt_bridge", bridge, logs)
        if ids:
            topics = [f"/{i}/{t}" for i in ids for t in FLEET_TOPICS] + [t for _, t in gt_map]
        else:
            topics = list(dict.fromkeys([*resolved["recording"]["topics"], GT_TOPIC]))
        if world.get("layout_data", {}).get("actors"):
            actors = Child("actors", [sys.executable, "-m", "qrsimbench.adapters.gazebo.actors",
                                      str(run_dir)], logs)
        if ids and resolved.get("events"):
            scen_events = Child("scenario_events", [sys.executable, "-m", "qrsimbench.runner.events",
                                                    str(run_dir)], logs)
        pol = resolved.get("fleet_policy") or {}
        if ids and pol.get("human_aware") and _has("navigation.dynamic_safety"):
            safety = Child("dynamic_safety", [sys.executable, "-m", "qrsimbench.navigation.dynamic_safety",
                                              str(run_dir)], logs)
        if ids and pol.get("awareness", True) and _has("navigation.fleet_awareness"):
            aware = Child("fleet_awareness", [sys.executable, "-m", "qrsimbench.navigation.fleet_awareness",
                                              *ids], logs)
        bag = Child("rosbag", ["ros2", "bag", "record", "-s", "mcap", "-o", str(run_dir / "bag"),
                               *topics], logs)
        log(f"simulator + recorder started ({len(ids) or 1} robot(s)); waiting for Nav2")
        timeout = resolved["mission"]["timeout_seconds"] + 240 + 30 * len(ids)  # + bringup allowance
        mgr = resolved.get("fleet_manager") or "qr"
        ext = fleet_managers.find(mgr) if ids and mgr != "qr" else None
        if ids and mgr != "qr" and ext is None:
            raise RuntimeError(f"fleet_manager {mgr!r} is not available in this build (no plugin registered "
                               "under the qrsimbench.fleet_managers entry point)")
        if ext is not None and mgr in getattr(ext, "WAIT_FOR_NAV2", ()):
            wait_nav2_ready(ids, timeout=120 + 30 * len(ids))
        if ext is not None:
            cmdrs = ext.start(mgr, resolved, run_dir, logs)
        else:
            cmdrs = [Child(f"mission_{i}" if i else "mission",
                           [sys.executable, "-m", "qrsimbench.navigation.mission", str(run_dir),
                            *([i] if i else [])], logs) for i in (ids or [None])]
        deadline = time.time() + timeout
        guard = None  # memory guard: one runaway process must not get the whole queue OOM-killed
        for c in cmdrs:
            if ext is not None and c.name in getattr(ext, "BACKGROUND", ()):
                continue
            while (guard is None and time.time() < deadline and c.wait(timeout=5.0) is None
                   and not _fleet_settled(run_dir, ids)):
                kids = [k for k in (sim, bag, gt, cams, aware, actors, safety, scen_events, *cmdrs, *guis)
                        if k]
                if mem_available_mb() < MIN_FREE_MB:
                    big = max(kids, key=lambda k: group_rss_mb(k.proc.pid))
                    guard = {"reason": "low_memory", "free_mb": round(mem_available_mb()),
                             "largest": big.name, "largest_mb": round(group_rss_mb(big.proc.pid))}
            if guard is not None:
                log(f"memory guard: {guard}; stopping this run")
                (run_dir / "infrastructure_failure.json").write_text(json.dumps(guard, indent=2))
                break
            if c.alive and _fleet_settled(run_dir, ids):
                log("all working robots finished; the broken-down robot's mission is not awaited")
                break
            if c.alive:
                log(f"{c.name} hung; stopping")
                c.stop()
        results = []
        for i in ids or [None]:
            f = run_dir / (f"mission_{i}.json" if i else "mission.json")
            results.append(json.loads(f.read_text()).get("result") if f.exists() else None)
        if ids:  # merged event log + a fleet-level mission.json for tools that read one file
            merged = []
            for ef in [run_dir / f"events_{i}.json" for i in ids] + [run_dir / "events_scenario.json"]:
                merged += json.loads(ef.read_text()) if ef.exists() else []
            (run_dir / "events.json").write_text(json.dumps(sorted(merged, key=lambda e: e["t"]), indent=2))
            ms = [json.loads((run_dir / f"mission_{i}.json").read_text()) for i in ids
                  if (run_dir / f"mission_{i}.json").exists()]
            starts = [m["mission_start"] for m in ms if m.get("mission_start")]
            ends = [m["mission_end"] for m in ms if m.get("mission_end")]
            (run_dir / "mission.json").write_text(json.dumps({
                "result": next((r for r in results if r != "SUCCEEDED"), "SUCCEEDED") if all(results)
                else "NAV_STACK_NOT_READY",
                "mission_start": min(starts) if starts else None, "mission_end": max(ends) if ends else None,
                "robots": dict(zip(ids, results, strict=True))}, indent=2))
        mission = json.loads((run_dir / "mission.json").read_text()) \
            if (run_dir / "mission.json").exists() else {}
        result = mission.get("result") or "NAV_STACK_NOT_READY"
        log(f"mission result: {result}" + (f" {dict(zip(ids, results, strict=True))}" if ids else ""))
        if record:
            time.sleep(3)  # hold the final pose in the video
    finally:
        for c in (cams, *cmdrs, scen_events, aware, safety, actors, bag, gt, *guis, sim):
            if c is not None:
                c.stop()
        log("children stopped")

    verdict = "FAIL"
    try:
        out = analyze(run_dir, resolved)
        verdict = out["verdict"]
    except Exception as e:  # keep the bundle, record the failure
        (run_dir / "metrics.json").write_text(json.dumps(
            {"verdict": "FAIL", "codes": ["ANALYSIS_FAILURE"], "error": repr(e)}, indent=2))
        log(f"analysis failed: {e!r}")
    manifest["verdict"] = verdict
    mf.finish_manifest(run_dir, manifest, result)
    if record:
        try:
            from qrsimbench.reporting.video import compose_run_video

            log(f"video: {compose_run_video(run_dir, resolved)}")
        except Exception as e:
            log(f"video encoding failed: {e!r}")
    try:
        from qrsimbench.reporting.html import render_report

        log(f"report: {render_report(run_dir)}")
    except Exception as e:
        log(f"report generation failed: {e!r}")
    log(f"{verdict}  ({run_dir})")
    return run_dir, verdict
