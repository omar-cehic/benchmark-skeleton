"""Skeleton API for the lab benchmark (draft for discussion).

Three roles, kept separate on purpose:

  Robot   benchmark side. Loads one robot model, defines what it can sense
          (observation spec) and what it accepts as commands (action spec).
          Written once per robot (G1, Trossen bimanual, LEAP hand), reused by
          every task for that robot.
  Task    benchmark side. Adds the rest of the world, samples a start state
          from a seed, and judges pass/fail. May read privileged sim state,
          because judging is the benchmark's job.
  Policy  user side. Sees only observations, returns actions. Never gets a
          Drake context, plant, or diagram.

The runner wires these together, runs the episode on CENIC, and returns a
plain result record.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ApplySimulatorConfig,
    Context,
    Diagram,
    DiagramBuilder,
    InputPort,
    MultibodyPlant,
    OutputPort,
    Simulator,
    SimulatorConfig,
)

Observation = dict[str, np.ndarray]


# ---------------------------------------------------------------- specs
@dataclass(frozen=True)
class ArraySpec:
    """Shape and bounds of one observation entry or of the action vector."""
    shape: tuple[int, ...]
    low: np.ndarray | float = -np.inf
    high: np.ndarray | float = np.inf
    description: str = ""


@dataclass(frozen=True)
class TaskInfo:
    """Everything a policy is told at the start of an episode."""
    task_name: str
    instruction: str                      # natural-language description
    robot_name: str
    control_period: float                 # seconds between step() calls
    action_spec: ArraySpec
    observation_spec: dict[str, ArraySpec]


# ---------------------------------------------------------------- robot
class Robot(ABC):
    """One real robot (G1, Trossen arms, LEAP hand).

    The action format should copy the real hardware's command format, and
    action_spec() limits should match the real motors, so nothing a policy
    sends is impossible on hardware. For the G1 that is 5 numbers per motor:
    target angle, target speed, stiffness (kp), damping (kd), extra torque.
    """
    name: str
    control_period: float                 # e.g. 0.02 for a 50 Hz policy

    @abstractmethod
    def add_to_plant(self, plant: MultibodyPlant) -> None:
        """Before Finalize: load the model and any bodies bolted to it."""

    @abstractmethod
    def wire(
        self, builder: DiagramBuilder, plant: MultibodyPlant
    ) -> tuple[InputPort, dict[str, OutputPort]]:
        """After Finalize: add low-level controllers and sensors.

        Returns the one input port the action goes into, and the output
        ports that make up the observation. Only these ports are exported,
        so they ARE the observation/action boundary. Anything not returned
        here is invisible to the policy.
        """

    @abstractmethod
    def action_spec(self) -> ArraySpec: ...

    @abstractmethod
    def observation_spec(self) -> dict[str, ArraySpec]: ...


# ---------------------------------------------------------------- task
@dataclass
class Outcome:
    done: bool = False
    success: bool = False
    info: dict[str, Any] = field(default_factory=dict)   # partial credit etc.


class Task(ABC):
    name: str                             # unique id, e.g. "g1/stand"
    instruction: str                      # the language description
    tags: tuple[str, ...] = ()            # e.g. ("locomotion", "g1")
    time_limit: float                     # seconds of sim time

    @abstractmethod
    def make_robot(self) -> Robot: ...

    def add_scene(self, plant: MultibodyPlant) -> None:
        """Before Finalize: ground, terrain, objects. Default: nothing."""

    @abstractmethod
    def reset(
        self, rng: np.random.Generator, plant: MultibodyPlant, plant_context: Context
    ) -> None:
        """Seed -> initial conditions. All randomness must come from rng."""

    @abstractmethod
    def evaluate(self, plant: MultibodyPlant, plant_context: Context) -> Outcome:
        """Called after every control step. Privileged state is allowed here.

        Return done=True to end early (success or unrecoverable failure).
        Reaching time_limit without success counts as failure.
        """

    def demo_policies(self) -> dict[str, Policy]:
        """{"success": ..., "failure": ...} sanity-check controllers."""
        return {}


# ---------------------------------------------------------------- policy
class Policy(ABC):
    def reset(self, info: TaskInfo) -> None:
        """Clear state from the previous episode."""

    @abstractmethod
    def step(self, obs: Observation) -> np.ndarray: ...


# ---------------------------------------------------------------- registry
_REGISTRY: dict[str, type[Task]] = {}


def register(cls: type[Task]) -> type[Task]:
    if cls.name in _REGISTRY:
        raise ValueError(f"duplicate task name {cls.name!r}")
    _REGISTRY[cls.name] = cls
    return cls


def list_tasks(robot: str | None = None, tags: tuple[str, ...] = ()) -> list[str]:
    """Supports 'run the full suite' and 'run a subset' (e.g. only G1)."""
    out = []
    for name, cls in sorted(_REGISTRY.items()):
        if robot and not name.startswith(robot + "/"):
            continue
        if not set(tags) <= set(cls.tags):
            continue
        out.append(name)
    return out


# ---------------------------------------------------------------- runner
DEFAULT_SIM = SimulatorConfig(integration_scheme="cenic", accuracy=1e-3)


@dataclass
class EpisodeResult:
    task: str
    seed: int
    success: bool
    sim_time: float
    wall_time: float
    steps: int
    think_time_mean: float                # seconds the policy took per step
    think_time_max: float
    info: dict[str, Any]

    @property
    def realtime_rate(self) -> float:
        return self.sim_time / self.wall_time if self.wall_time > 0 else float("inf")


@dataclass
class _Built:
    diagram: Diagram
    plant: MultibodyPlant
    robot: Robot


def build(task: Task) -> _Built:
    builder = DiagramBuilder()
    # time_step=0 -> continuous plant, which CENIC requires.
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    robot = task.make_robot()
    robot.add_to_plant(plant)
    task.add_scene(plant)
    plant.Finalize()
    action_port, sensor_ports = robot.wire(builder, plant)
    builder.ExportInput(action_port, "action")
    for key, port in sensor_ports.items():
        builder.ExportOutput(port, f"obs/{key}")
    return _Built(builder.Build(), plant, robot)


def run_episode(
    task: Task,
    policy: Policy,
    seed: int,
    sim_config: SimulatorConfig = DEFAULT_SIM,
    built: _Built | None = None,
) -> EpisodeResult:
    b = built or build(task)
    diagram, plant, robot = b.diagram, b.plant, b.robot

    simulator = Simulator(diagram)
    ApplySimulatorConfig(sim_config, simulator)
    ctx = simulator.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(ctx)

    task.reset(np.random.default_rng(seed), plant, plant_ctx)
    spec = robot.action_spec()
    action_in = diagram.GetInputPort("action")
    action_in.FixValue(ctx, np.zeros(spec.shape))
    simulator.Initialize()

    obs_ports = {
        diagram.get_output_port(i).get_name()[len("obs/"):]: diagram.get_output_port(i)
        for i in range(diagram.num_output_ports())
    }
    policy.reset(TaskInfo(task.name, task.instruction, robot.name,
                          robot.control_period, spec, robot.observation_spec()))

    dt = robot.control_period
    n_steps = int(round(task.time_limit / dt))
    outcome, k = Outcome(), 0
    think_times = []
    t0 = time.perf_counter()
    for k in range(1, n_steps + 1):
        # Copies, so the policy can't hold a live reference into the sim.
        obs = {key: np.array(p.Eval(ctx), copy=True) for key, p in obs_ports.items()}
        t_think = time.perf_counter()
        action = np.asarray(policy.step(obs), dtype=float)
        think_times.append(time.perf_counter() - t_think)
        if action.shape != spec.shape:
            raise ValueError(f"action shape {action.shape} != {spec.shape}")
        action_in.FixValue(ctx, np.clip(action, spec.low, spec.high))
        # Physics is frozen while the policy thinks (repeatable default).
        # A later "realistic" mode could apply the action late instead.
        simulator.AdvanceTo(k * dt)        # action held constant until next step
        outcome = task.evaluate(plant, plant_ctx)
        if outcome.done:
            break
    wall = time.perf_counter() - t0

    return EpisodeResult(task.name, seed, outcome.done and outcome.success,
                         ctx.get_time(), wall, k, float(np.mean(think_times)),
                         float(np.max(think_times)), outcome.info)


def check_demos(task: Task, seeds=range(5)) -> dict[str, list[bool]]:
    """Success demo should pass every seed, failure demo should fail every seed.
    Doubles as a regression test when Drake/CENIC versions change."""
    b = build(task)
    return {kind: [run_episode(task, pol, s, built=b).success for s in seeds]
            for kind, pol in task.demo_policies().items()}
