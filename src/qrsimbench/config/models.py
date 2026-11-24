"""Scenario contract (spec §11) as Pydantic models."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from qrsimbench import SCHEMA_VERSION

KNOWN_METRICS = {
    "success",
    "time_to_goal",
    "path_length",
    "path_efficiency",
    "min_clearance",
    "stopped_time",
    "stop_episodes",
    "goal_error",
    "mean_speed",
    "max_speed",
    "localization_error",
}


class Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


class Experiment(Strict):
    name: str = Field(pattern=r"^[a-z0-9_]+$")
    description: str = ""
    seed: int = 0


class Runtime(Strict):
    ros_distro: Literal["jazzy"] = "jazzy"
    simulator: Literal["gazebo_harmonic"] = "gazebo_harmonic"
    headless: bool = False
    # Nav2 bringup per fleet robot: "stock" (nav2_bringup) or "slim" (simulation/launch/qr_bringup_slim:
    # controller, planner, behaviors and BT navigator only), for large fleets on one machine.
    nav_launch: Literal["stock", "slim"] = "stock"
    bringup_stagger_s: float = Field(default=6.0, ge=0)  # s between fleet robots' Nav2 bring-up


class Robot(Strict):
    profile: str


class World(Strict):
    id: str
    actor_speed_scale: float = Field(default=1.0, gt=0)  # scales moving workers / vehicles
    non_yielding_actors: list[str] = Field(default_factory=list)  # actors that never yield to robots
    actor_yield_probability: float = Field(default=1.0, ge=0, le=1)  # crowd: yield per encounter
    vehicles_yield_to_workers: bool = False  # forklifts also stop for workers (off: benchmark runs)


class Navigation(Strict):
    stack: Literal["nav2"] = "nav2"
    parameter_profile: str = "nav2_default_v1"
    # Scenario-level Nav2 parameter overrides, merged over the profile (e.g. speed limits in a sweep).
    overrides: dict = Field(default_factory=dict)


class Pose(Strict):
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


class Mission(Strict):
    # A named station from the world profile, or an explicit pose. Single-robot scenarios set
    # start/goal here; fleet scenarios set them per robot under `fleet` and only use the timeout.
    start: str | Pose | None = None
    goal: str | Pose | None = None
    timeout_seconds: float = Field(default=180, gt=0)


class FleetMember(Strict):
    """One robot in a fleet scenario. Goals are visited in order (a transport task chain)."""

    id: str = Field(pattern=r"^[a-z][a-z0-9_]*$")
    start: str | Pose
    goals: list[str | Pose] = Field(min_length=1)
    profile: str | None = None  # defaults to robot.profile


class _GenericFleetPolicy(BaseModel):
    """Fleet policy for builds without a fleet layer plugin: generic switches, plugin keys pass through."""

    model_config = ConfigDict(extra="allow")
    awareness: bool = True      # share robot positions into each robot's costmaps (needs a fleet layer)
    reservations: bool = True   # let the fleet layer coordinate goals (needs a fleet layer)
    stagger_s: float = Field(default=4.0, ge=0)  # departure stagger between robots
    retries: int = Field(default=2, ge=0)        # goal retries after a failed leg
    human_aware: bool = False   # people-aware speed limiting (needs a fleet layer)


class ElapsedTimeTrigger(Strict):
    type: Literal["elapsed_time"]
    seconds: float = Field(ge=0)


class RobotNearTrigger(Strict):
    """Fires when the robot (map frame, from Nav2 feedback) comes within radius_m of (x, y)."""

    type: Literal["robot_near"]
    x: float
    y: float
    radius_m: float = Field(default=0.5, gt=0)
    robot: str | None = None  # fleet robot id; default = the first (or only) robot


class SpawnEntityAction(Strict):
    type: Literal["spawn_entity"]
    asset: str
    pose: Pose
    name: str | None = None


class DeleteEntityAction(Strict):
    type: Literal["delete_entity"]
    name: str


class DisableRobotAction(Strict):
    """Robot breakdown: the robot's drive stops responding for the rest of the run."""

    type: Literal["disable_robot"]
    robot: str


class RelocalizeRobotAction(Strict):
    """Localization kick: re-initialise the robot's localization with an offset from its estimate."""

    type: Literal["relocalize_robot"]
    robot: str
    dx: float = 1.0
    dy: float = 0.0
    dyaw: float = 0.0


class Event(Strict):
    id: str
    trigger: ElapsedTimeTrigger | RobotNearTrigger = Field(discriminator="type")
    action: SpawnEntityAction | DeleteEntityAction | DisableRobotAction | RelocalizeRobotAction = Field(
        discriminator="type")


class Recording(Strict):
    storage: Literal["mcap"] = "mcap"
    topics: list[str] = Field(
        default_factory=lambda: ["/tf", "/tf_static", "/odom", "/scan", "/cmd_vel", "/plan", "/amcl_pose"]
    )

    @field_validator("topics")
    @classmethod
    def _topics_absolute(cls, v: list[str]) -> list[str]:
        bad = [t for t in v if not t.startswith("/")]
        if bad:
            raise ValueError(f"topics must start with '/': {bad}")
        return v


class Threshold(Strict):
    equals: bool | None = None
    greater_than_m: float | None = None
    warn_below_m: float | None = None
    less_than_seconds: float | None = None


try:  # QR fleet layer (closed builds); see navigation/policy.py
    from qrsimbench.navigation.policy import FleetPolicy
except ImportError:  # open builds
    FleetPolicy = _GenericFleetPolicy


class Scenario(Strict):
    schema_version: str = SCHEMA_VERSION
    experiment: Experiment
    runtime: Runtime = Field(default_factory=Runtime)
    robot: Robot
    world: World
    navigation: Navigation = Field(default_factory=Navigation)
    mission: Mission
    fleet: list[FleetMember] = Field(default_factory=list)
    fleet_policy: FleetPolicy = Field(default_factory=FleetPolicy)
    # Who coordinates the fleet: QR's commanders ("qr"), Open-RMF ("rmf"), free_fleet ("free_fleet") or
    # the MAPF baseline ("mapf": prioritized planning + action-dependency execution).
    fleet_manager: Literal["qr", "rmf", "free_fleet", "mapf"] = "qr"
    events: list[Event] = Field(default_factory=list)
    recording: Recording = Field(default_factory=Recording)
    metrics: list[str] = Field(default_factory=lambda: sorted(KNOWN_METRICS))
    gates: dict[str, Threshold] = Field(default_factory=dict)

    @field_validator("schema_version")
    @classmethod
    def _schema_supported(cls, v: str) -> str:
        if v != SCHEMA_VERSION:
            raise ValueError(f"unsupported schema_version {v!r}; this build supports {SCHEMA_VERSION!r}")
        return v

    @field_validator("metrics")
    @classmethod
    def _metrics_known(cls, v: list[str]) -> list[str]:
        unknown = sorted(set(v) - KNOWN_METRICS)
        if unknown:
            raise ValueError(f"unknown metrics {unknown}; known: {sorted(KNOWN_METRICS)}")
        return v

    @model_validator(mode="after")
    def _mission_or_fleet(self) -> Scenario:
        if not self.fleet and (self.mission.start is None or self.mission.goal is None):
            raise ValueError("mission.start and mission.goal are required unless `fleet` is given")
        ids = [m.id for m in self.fleet]
        if len(ids) != len(set(ids)):
            raise ValueError(f"duplicate fleet robot ids: {ids}")
        return self

    @model_validator(mode="after")
    def _unique_event_ids(self) -> Scenario:
        ids = [e.id for e in self.events]
        dupes = sorted({i for i in ids if ids.count(i) > 1})
        if dupes:
            raise ValueError(f"duplicate event ids: {dupes}")
        return self
