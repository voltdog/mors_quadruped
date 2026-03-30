#!/usr/bin/env python3

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "Simulator"
LCM_MSG_ROOT = REPO_ROOT / "lcm_msgs"
LCM_DEFAULT_URL = "udpm://239.255.76.67:7667?ttl=0"
BOOTSTRAP_ENV = "WBIC_TUNE_BOOTSTRAPPED"


def _ensure_runtime() -> None:
    try:
        import mujoco.viewer  # noqa: F401
        return
    except ModuleNotFoundError:
        venv_python = REPO_ROOT / ".mpc_venv" / "bin" / "python"
        current_python = Path(sys.executable)
        if (
            os.environ.get(BOOTSTRAP_ENV) != "1"
            and venv_python.exists()
            and current_python != venv_python
        ):
            env = os.environ.copy()
            env[BOOTSTRAP_ENV] = "1"
            os.execvpe(
                str(venv_python),
                [str(venv_python), __file__, *sys.argv[1:]],
                env,
            )
        raise


_ensure_runtime()

import lcm
import mujoco.viewer
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


class _DummyViewer:
    def sync(self) -> None:
        pass

    def close(self) -> None:
        pass


# The passive MuJoCo viewer is useful for manual runs but blocks automation.
mujoco.viewer.launch_passive = lambda *args, **kwargs: _DummyViewer()


sys.path.insert(0, str(SIM_ROOT))
sys.path.insert(0, str(LCM_MSG_ROOT))

from mors_simulator import Hardware_Level_Sim  # noqa: E402
from mors_msgs.enable_msg import enable_msg  # noqa: E402
from mors_msgs.gait_params_msg import gait_params_msg  # noqa: E402
from mors_msgs.phase_signal_msg import phase_signal_msg  # noqa: E402
from mors_msgs.robot_cmd_msg import robot_cmd_msg  # noqa: E402
from mors_msgs.wbc_cmd_msg import wbc_cmd_msg  # noqa: E402


TAU_LIMITS = np.array([6.0, 6.0, 12.0] * 4, dtype=float)
ACTIVE_LEGS = [True, True, True, True]
WARMUP_DURATION = 0.5
WBIC_CONFIG = "wbic.yaml"
SWING_CONFIG = "swing_controller.yaml"
STANCE_CONFIG = "stance_controller_mpc.yaml"
TROT = [0.0, 0.5, 0.5, 0.0]
PACE = [0.0, 0.5, 0.0, 0.5]
WALK_DIAG_A = [0.0, 0.5, 0.25, 0.75]
WALK_DIAG_B = [0.0, 0.5, 0.75, 0.25]
WALK_WAVE_A = [0.0, 0.25, 0.5, 0.75]
WALK_WAVE_B = [0.0, 0.75, 0.5, 0.25]
GAIT_LIBRARY: list[tuple[str, list[float]]] = [
    ("trot", TROT),
    ("pace", PACE),
    ("walk_diag_a", WALK_DIAG_A),
    ("walk_diag_b", WALK_DIAG_B),
    ("walk_wave_a", WALK_WAVE_A),
    ("walk_wave_b", WALK_WAVE_B),
]
SWING_PHASES = {0, 2}
STANCE_PHASES = {1, 3}
STAIRS_SCREEN_SPEED = 0.18
STAIRS_FINAL_SPEED = 0.20
STAIRS_MARGIN_SPEED = 0.18
ScheduleFn = Callable[[float, dict[str, Any]], dict[str, Any]]


@dataclass
class StairsProfile:
    front_x: float
    top_x: float
    top_z: float


def load_stairs_profile() -> StairsProfile:
    scene_path = REPO_ROOT / "Simulator" / "MJCF" / "stairs.xml"
    if not scene_path.exists():
        return StairsProfile(front_x=0.5, top_x=2.3, top_z=0.25)

    root = ET.fromstring(scene_path.read_text(encoding="utf-8"))
    front_x = math.inf
    top_x = -math.inf
    top_z = 0.0
    for geom in root.findall(".//geom"):
        name = geom.attrib.get("name", "")
        if not name.startswith("step_"):
            continue
        pos = [float(value) for value in geom.attrib["pos"].split()]
        size = [float(value) for value in geom.attrib["size"].split()]
        front_x = min(front_x, pos[0] - size[0])
        top_x = max(top_x, pos[0] + size[0])
        top_z = max(top_z, pos[2] + size[2])

    if not math.isfinite(front_x) or not math.isfinite(top_x):
        return StairsProfile(front_x=0.5, top_x=2.3, top_z=0.25)
    return StairsProfile(front_x=front_x, top_x=top_x, top_z=top_z)


STAIRS_PROFILE = load_stairs_profile()
STAIRS_TARGET_X = max(
    STAIRS_PROFILE.front_x + 0.45,
    min(STAIRS_PROFILE.front_x + 0.85, STAIRS_PROFILE.top_x - 0.65),
)
STAIRS_TARGET_BODY_Z = 0.28


@dataclass
class ValidationScenario:
    name: str
    schedule_fn: ScheduleFn
    duration: float


@dataclass
class CandidateResult:
    tag: str
    params: dict[str, Any]
    stable: bool
    score: float
    metrics: dict[str, Any]
    details: list[dict[str, Any]] | None = None


class TelemetrySubscriber:
    def __init__(self, lcm_url: str):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._lcm = lcm.LCM(lcm_url)
        self._lcm.subscribe("WBC_CMD", self._on_wbc_cmd)
        self._lcm.subscribe("GAIT_PHASE", self._on_phase)
        self.last_wbc_cmd: wbc_cmd_msg | None = None
        self.last_phase: phase_signal_msg | None = None
        self.last_wbc_ts = 0.0
        self.last_phase_ts = 0.0
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while not self._stop.is_set():
            self._lcm.handle_timeout(50)

    def _on_wbc_cmd(self, channel: str, data: bytes) -> None:
        msg = wbc_cmd_msg.decode(data)
        with self._lock:
            self.last_wbc_cmd = msg
            self.last_wbc_ts = time.monotonic()

    def _on_phase(self, channel: str, data: bytes) -> None:
        msg = phase_signal_msg.decode(data)
        with self._lock:
            self.last_phase = msg
            self.last_phase_ts = time.monotonic()

    def snapshot(self) -> tuple[wbc_cmd_msg | None, phase_signal_msg | None, float, float]:
        with self._lock:
            return self.last_wbc_cmd, self.last_phase, self.last_wbc_ts, self.last_phase_ts

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)


class CommandPublisher:
    def __init__(self, lcm_url: str):
        self._lcm = lcm.LCM(lcm_url)

    def publish(self, command: dict[str, Any]) -> None:
        en = enable_msg()
        enabled = bool(command["enabled"])
        en.locomotion_en = enabled
        en.locomotion_reset = False
        en.leg_controller_en = enabled
        en.leg_controller_reset = False

        gait = gait_params_msg()
        gait.standing = bool(command["standing"])
        gait.gait_type = list(command["gait_type"])
        gait.t_sw = float(command["t_sw"])
        gait.t_st = float(command["t_st"])
        gait.stride_height = float(command["stride_height"])

        robot = robot_cmd_msg()
        robot.active_legs = ACTIVE_LEGS[:]
        robot.adaptation_type = int(command.get("adaptation_type", 0))
        robot.cmd_vel = [
            float(command["lin_vel"][0]),
            float(command["lin_vel"][1]),
            float(command["lin_vel"][2]),
            float(command["ang_vel"][0]),
            float(command["ang_vel"][1]),
            float(command["ang_vel"][2]),
        ]
        robot.cmd_pose = [
            float(command["pos"][0]),
            float(command["pos"][1]),
            float(command["pos"][2]),
            float(command["orientation"][0]),
            float(command["orientation"][1]),
            float(command["orientation"][2]),
        ]

        self._lcm.publish("ENABLE", en.encode())
        self._lcm.publish("GAIT_PARAMS", gait.encode())
        self._lcm.publish("ROBOT_CMD", robot.encode())


class ManagedSimulator:
    def __init__(self):
        self.sim = Hardware_Level_Sim()
        self.sim.foot_positions_type = "global"
        self._zero_references()

    def _zero_references(self) -> None:
        self.sim.ref_joint_pos = [0.0] * 12
        self.sim.ref_joint_vel = [0.0] * 12
        self.sim.ref_joint_torq = [0.0] * 12
        self.sim.kp = [0.0] * 12
        self.sim.kd = [0.0] * 12

    def reset(self) -> None:
        try:
            self.sim.env.close()
        except Exception:
            pass
        self._zero_references()
        self.sim.cur_joint_pos = [0.0] * 12
        self.sim.cur_joint_vel = [0.0] * 12
        self.sim.cur_joint_torq = [0.0] * 12
        self.sim.body_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.sim.body_lin_pos = [0.0] * 3
        self.sim.body_ang_vel = [0.0] * 3
        self.sim.body_lin_vel = np.zeros(3)
        self.sim.body_lin_acc = np.zeros(3)
        self.sim.imu_data = [0.0] * 13
        self.sim.contact_flags = [False] * 4
        self.sim.yaw = 0.0
        self.sim.pre_yaw = 0.0
        self.sim.first_yaw = True
        self.sim.offset_yaw = 0.0
        self.sim.yaw_final = 0.0
        self.sim.first_step = True
        self.sim.sim_it = 0
        self.sim.init_simulation()

    def step(self) -> None:
        self.sim.loop()

    def foot_positions_global(self) -> np.ndarray:
        return np.asarray(self.sim.env.get_global_foot_positions(), dtype=float)

    def actuation(self) -> np.ndarray:
        ref_pos = np.asarray(self.sim.ref_joint_pos, dtype=float)
        ref_vel = np.asarray(self.sim.ref_joint_vel, dtype=float)
        ref_tau = np.asarray(self.sim.ref_joint_torq, dtype=float)
        kp = np.asarray(self.sim.kp, dtype=float)
        kd = np.asarray(self.sim.kd, dtype=float)
        cur_pos = np.asarray(self.sim.cur_joint_pos, dtype=float)
        cur_vel = np.asarray(self.sim.cur_joint_vel, dtype=float)
        return kp * (ref_pos - cur_pos) + kd * (ref_vel - cur_vel) + ref_tau


class ControllerRunner:
    def __init__(self, config_path: Path, lcm_url: str, log_dir: Path):
        env = os.environ.copy()
        env["CONFIGPATH"] = str(config_path)
        env["LCM_DEFAULT_URL"] = lcm_url
        self.processes = [
            subprocess.Popen(
                [str(REPO_ROOT / "LocomotionController/build/locomotionControllerMPC")],
                env=env,
                stdout=(log_dir / "locomotion.log").open("w"),
                stderr=subprocess.STDOUT,
            ),
        ]

    def running(self) -> bool:
        return all(process.poll() is None for process in self.processes)

    def stop(self) -> None:
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
        time.sleep(0.2)
        for process in self.processes:
            if process.poll() is None:
                process.kill()
        for process in self.processes:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def smooth_step(value: float, duration: float) -> float:
    if duration <= 1e-9:
        return 1.0
    ratio = clamp(value / duration, 0.0, 1.0)
    return ratio * ratio * (3.0 - 2.0 * ratio)


def quat_angle(actual_xyzw: np.ndarray, desired_xyzw: np.ndarray) -> float:
    q_act = Rotation.from_quat(actual_xyzw)
    q_des = Rotation.from_quat(desired_xyzw)
    return float((q_des * q_act.inv()).magnitude())


def _yaml_load(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _float_list(values: list[Any]) -> list[float]:
    return [float(value) for value in values]


def normalize_wbic_config(config: dict[str, Any]) -> dict[str, float]:
    ordered_keys = [
        "Qf_entry",
        "Qa_entry",
        "body_ori_task_kp",
        "body_ori_task_kd",
        "body_pos_task_kp",
        "body_pos_task_kd",
        "tip_pos_task_kp",
        "tip_pos_task_kd",
        "joint_kp_stance",
        "joint_kd_stance",
        "joint_kp_swing",
        "joint_kd_swing",
    ]
    return {key: float(config[key]) for key in ordered_keys}


def normalize_swing_config(config: dict[str, Any]) -> dict[str, Any]:
    return {
        "interleave_x": _float_list(config["interleave_x"]),
        "interleave_y": _float_list(config["interleave_y"]),
        "dz_near_ground": float(config["dz_near_ground"]),
        "k1_fsp": float(config["k1_fsp"]),
    }


def normalize_stance_config(config: dict[str, Any]) -> dict[str, Any]:
    return {
        "horizon": int(config["horizon"]),
        "friction": float(config["friction"]),
        "f_min": float(config["f_min"]),
        "f_max": float(config["f_max"]),
        "Q": _float_list(config["Q"]),
        "R": _float_list(config["R"]),
        "Q_gain": float(config["Q_gain"]),
        "R_gain": float(config["R_gain"]),
    }


def normalize_gait_config(config: dict[str, Any]) -> dict[str, Any]:
    gait_type = [float(value) % 1.0 for value in config["gait_type"]]
    return {
        "name": str(config.get("name", "custom")),
        "gait_type": gait_type,
        "t_sw": float(config["t_sw"]),
        "t_st": float(config["t_st"]),
        "stride_height": float(config["stride_height"]),
    }


def default_gait_config() -> dict[str, Any]:
    return normalize_gait_config(
        {
            "name": "trot",
            "gait_type": TROT,
            "t_sw": 0.21,
            "t_st": 0.3,
            "stride_height": 0.08,
        }
    )


def normalize_candidate(params: dict[str, Any]) -> dict[str, Any]:
    normalized = {
        "wbic": normalize_wbic_config(params["wbic"]),
        "swing_controller": normalize_swing_config(params["swing_controller"]),
        "stance_controller_mpc": normalize_stance_config(params["stance_controller_mpc"]),
    }
    if "gait" in params:
        normalized["gait"] = normalize_gait_config(params["gait"])
    return normalized


def load_base_params() -> dict[str, Any]:
    config_root = REPO_ROOT / "config"
    params = {
        "wbic": _yaml_load(config_root / WBIC_CONFIG),
        "swing_controller": _yaml_load(config_root / SWING_CONFIG),
        "stance_controller_mpc": _yaml_load(config_root / STANCE_CONFIG),
        "gait": default_gait_config(),
    }
    return normalize_candidate(params)


def workspace_for_candidate(params: dict[str, Any]) -> tempfile.TemporaryDirectory[str]:
    tmpdir = tempfile.TemporaryDirectory(prefix="wbic_tune_")
    tmp_path = Path(tmpdir.name)
    shutil.copytree(REPO_ROOT / "config", tmp_path / "config")
    os.symlink(REPO_ROOT / "common", tmp_path / "common")

    with (tmp_path / "config" / WBIC_CONFIG).open("w", encoding="utf-8") as handle:
        yaml.safe_dump(params["wbic"], handle, sort_keys=False)
    with (tmp_path / "config" / SWING_CONFIG).open("w", encoding="utf-8") as handle:
        yaml.safe_dump(params["swing_controller"], handle, sort_keys=False)
    with (tmp_path / "config" / STANCE_CONFIG).open("w", encoding="utf-8") as handle:
        yaml.safe_dump(params["stance_controller_mpc"], handle, sort_keys=False)
    return tmpdir


def make_command(
    *,
    enabled: bool,
    standing: bool,
    gait_type: list[float] | None,
    pos: list[float],
    orientation: list[float],
    lin_vel: list[float],
    ang_vel: list[float],
    stride_height: float,
    t_sw: float,
    t_st: float,
) -> dict[str, Any]:
    return {
        "enabled": enabled,
        "standing": standing,
        "gait_type": list(TROT if gait_type is None else gait_type),
        "t_sw": t_sw,
        "t_st": t_st,
        "stride_height": stride_height,
        "pos": pos,
        "orientation": orientation,
        "lin_vel": lin_vel,
        "ang_vel": ang_vel,
    }


def warmup_command() -> dict[str, Any]:
    return make_command(
        enabled=True,
        standing=True,
        gait_type=TROT,
        pos=[0.0, 0.0, 0.2],
        orientation=[0.0, 0.0, 0.0],
        lin_vel=[0.0, 0.0, 0.0],
        ang_vel=[0.0, 0.0, 0.0],
        stride_height=0.06,
        t_sw=0.2,
        t_st=0.2,
    )


def disabled_command() -> dict[str, Any]:
    command = warmup_command()
    command["enabled"] = False
    return command


def candidate_gait(params: dict[str, Any]) -> dict[str, Any]:
    return normalize_gait_config(params.get("gait", default_gait_config()))


def build_stairs_schedule(forward_speed: float) -> ScheduleFn:
    def schedule(t: float, params: dict[str, Any]) -> dict[str, Any]:
        gait = candidate_gait(params)
        if t < 1.0:
            return warmup_command()
        if t < 1.8:
            alpha = smooth_step(t - 1.0, 0.4)
            return make_command(
                enabled=True,
                standing=True,
                gait_type=gait["gait_type"],
                pos=[0.0, 0.0, 0.2 + 0.006 * alpha],
                orientation=[0.0, 0.0, 0.0],
                lin_vel=[0.0, 0.0, 0.0],
                ang_vel=[0.0, 0.0, 0.0],
                stride_height=gait["stride_height"],
                t_sw=gait["t_sw"],
                t_st=gait["t_st"],
            )
        alpha = smooth_step(t - 1.8, 0.8)
        return make_command(
            enabled=True,
            standing=False,
            gait_type=gait["gait_type"],
            pos=[0.0, 0.0, 0.2],
            orientation=[0.0, 0.0, 0.0],
            lin_vel=[forward_speed * alpha, 0.0, 0.0],
            ang_vel=[0.0, 0.0, 0.0],
            stride_height=gait["stride_height"],
            t_sw=gait["t_sw"],
            t_st=gait["t_st"],
        )
    return schedule


short_schedule = build_stairs_schedule(STAIRS_SCREEN_SPEED)
final_schedule = build_stairs_schedule(STAIRS_FINAL_SPEED)
margin_schedule = build_stairs_schedule(STAIRS_MARGIN_SPEED)


def infer_damping_ratio(kp: float, kd: float) -> float:
    return kd / max(2.0 * math.sqrt(max(kp, 1e-9)), 1e-9)


def clamp_wbic_config(config: dict[str, Any]) -> dict[str, float]:
    wbic = copy.deepcopy(config)
    wbic["Qa_entry"] = clamp(float(wbic["Qa_entry"]), 3.0, 24.0)
    wbic["Qf_entry"] = clamp(float(wbic["Qf_entry"]), 0.05, 0.8)

    body_ori_kp = clamp(float(wbic["body_ori_task_kp"]), 130.0, 420.0)
    body_pos_kp = clamp(float(wbic["body_pos_task_kp"]), 120.0, 420.0)
    tip_kp = clamp(float(wbic["tip_pos_task_kp"]), 220.0, 700.0)

    body_ori_zeta = clamp(
        infer_damping_ratio(body_ori_kp, float(wbic["body_ori_task_kd"])), 0.8, 1.8
    )
    body_pos_zeta = clamp(
        infer_damping_ratio(body_pos_kp, float(wbic["body_pos_task_kd"])), 0.9, 1.9
    )
    tip_zeta = clamp(
        infer_damping_ratio(tip_kp, float(wbic["tip_pos_task_kd"])), 0.35, 1.0
    )

    wbic["body_ori_task_kp"] = body_ori_kp
    wbic["body_pos_task_kp"] = body_pos_kp
    wbic["tip_pos_task_kp"] = tip_kp
    wbic["body_ori_task_kd"] = clamp(2.0 * body_ori_zeta * math.sqrt(body_ori_kp), 16.0, 100.0)
    wbic["body_pos_task_kd"] = clamp(2.0 * body_pos_zeta * math.sqrt(body_pos_kp), 18.0, 110.0)
    wbic["tip_pos_task_kd"] = clamp(2.0 * tip_zeta * math.sqrt(tip_kp), 8.0, 60.0)

    wbic["joint_kp_stance"] = clamp(float(wbic["joint_kp_stance"]), 10.0, 34.0)
    wbic["joint_kd_stance"] = clamp(float(wbic["joint_kd_stance"]), 0.12, 0.8)
    wbic["joint_kp_swing"] = clamp(float(wbic["joint_kp_swing"]), 10.0, 30.0)
    wbic["joint_kd_swing"] = clamp(float(wbic["joint_kd_swing"]), 0.10, 0.8)
    return normalize_wbic_config(wbic)


def clamp_swing_config(config: dict[str, Any]) -> dict[str, Any]:
    swing = copy.deepcopy(config)
    x_front = clamp(float(np.mean(swing["interleave_x"][:2])), -0.02, 0.03)
    x_rear = clamp(float(np.mean(swing["interleave_x"][2:])), -0.02, 0.03)
    y_value = clamp(float(np.mean(swing["interleave_y"])), 0.05, 0.11)
    swing["interleave_x"] = [x_front, x_front, x_rear, x_rear]
    swing["interleave_y"] = [y_value, y_value, y_value, y_value]
    swing["dz_near_ground"] = -clamp(abs(float(swing["dz_near_ground"])), 0.005, 0.08)
    swing["k1_fsp"] = clamp(float(swing["k1_fsp"]), 0.005, 0.08)
    return normalize_swing_config(swing)


def _scale_q_groups(
    q_base: list[float],
    *,
    ori_scale: float,
    yaw_scale: float,
    pos_xy_scale: float,
    pos_z_scale: float,
    ang_xy_scale: float,
    yaw_rate_scale: float,
    lin_xy_scale: float,
    lin_z_scale: float,
) -> list[float]:
    q = [float(value) for value in q_base]
    q[0] *= ori_scale
    q[1] *= ori_scale
    q[2] *= yaw_scale
    q[3] *= pos_xy_scale
    q[4] *= pos_xy_scale
    q[5] *= pos_z_scale
    q[6] *= ang_xy_scale
    q[7] *= ang_xy_scale
    q[8] *= yaw_rate_scale
    q[9] *= lin_xy_scale
    q[10] *= lin_xy_scale
    q[11] *= lin_z_scale
    q[12] = float(q_base[12])
    return q


def _scale_r_groups(r_base: list[float], *, xy_scale: float, z_scale: float) -> list[float]:
    r: list[float] = []
    for leg_idx in range(4):
        offset = 3 * leg_idx
        r.append(float(r_base[offset]) * xy_scale)
        r.append(float(r_base[offset + 1]) * xy_scale)
        r.append(float(r_base[offset + 2]) * z_scale)
    return r


def clamp_stance_config(config: dict[str, Any]) -> dict[str, Any]:
    stance = copy.deepcopy(config)
    stance["horizon"] = int(round(clamp(float(stance["horizon"]), 10.0, 20.0)))
    stance["friction"] = clamp(float(stance["friction"]), 0.22, 0.5)
    stance["f_min"] = clamp(float(stance["f_min"]), 2.0, 12.0)
    stance["f_max"] = clamp(float(stance["f_max"]), 50.0, 120.0)
    stance["Q_gain"] = clamp(float(stance["Q_gain"]), 250.0, 3000.0)
    stance["R_gain"] = clamp(float(stance["R_gain"]), 1.5e-4, 3.0e-3)
    stance["Q"] = [clamp(float(value), 0.0, 1200.0) for value in stance["Q"]]
    stance["R"] = [clamp(float(value), 0.2, 3.0) for value in stance["R"]]
    stance["Q"][12] = 0.0
    return normalize_stance_config(stance)


def apply_search_constraints(
    params: dict[str, Any],
    base: dict[str, Any],
    *,
    freeze_swing: bool = False,
    lock_f_max: bool = False,
) -> dict[str, Any]:
    constrained = copy.deepcopy(params)
    if freeze_swing:
        constrained["swing_controller"] = copy.deepcopy(base["swing_controller"])
    if lock_f_max:
        constrained["stance_controller_mpc"]["f_max"] = float(base["stance_controller_mpc"]["f_max"])
    return normalize_candidate(constrained)


def gait_from_library(name: str, gait_type: list[float], t_st: float, t_sw: float, stride_height: float) -> dict[str, Any]:
    return normalize_gait_config(
        {
            "name": name,
            "gait_type": gait_type,
            "t_st": t_st,
            "t_sw": t_sw,
            "stride_height": stride_height,
        }
    )


def random_gait_config(rng: random.Random) -> dict[str, Any]:
    gait_name, gait_type = rng.choice(GAIT_LIBRARY)
    if gait_name.startswith("walk"):
        t_sw = rng.uniform(0.24, 0.36)
        t_st = rng.uniform(0.46, 0.72)
        stride_height = rng.uniform(0.10, 0.16)
    elif gait_name == "pace":
        t_sw = rng.uniform(0.22, 0.30)
        t_st = rng.uniform(0.36, 0.55)
        stride_height = rng.uniform(0.09, 0.14)
    else:
        t_sw = rng.uniform(0.20, 0.28)
        t_st = rng.uniform(0.30, 0.48)
        stride_height = rng.uniform(0.08, 0.13)
    return gait_from_library(gait_name, gait_type, t_st, t_sw, stride_height)


def local_gait_config(center: dict[str, Any], rng: random.Random) -> dict[str, Any]:
    gait = copy.deepcopy(center)
    gait["t_sw"] = clamp(float(gait["t_sw"]) * rng.uniform(0.92, 1.08), 0.18, 0.38)
    gait["t_st"] = clamp(float(gait["t_st"]) * rng.uniform(0.92, 1.08), 0.28, 0.80)
    gait["stride_height"] = clamp(float(gait["stride_height"]) * rng.uniform(0.92, 1.10), 0.07, 0.18)
    return normalize_gait_config(gait)


def build_candidate(base: dict[str, Any], rng: random.Random) -> dict[str, Any]:
    params = copy.deepcopy(base)

    wbic = params["wbic"]
    body_ori_kp = clamp(base["wbic"]["body_ori_task_kp"] * rng.uniform(0.85, 1.5), 130.0, 420.0)
    body_pos_kp = clamp(base["wbic"]["body_pos_task_kp"] * rng.uniform(0.80, 1.7), 120.0, 420.0)
    tip_kp = clamp(base["wbic"]["tip_pos_task_kp"] * rng.uniform(0.85, 2.0), 220.0, 700.0)
    body_ori_zeta = rng.uniform(0.95, 1.55)
    body_pos_zeta = rng.uniform(1.00, 1.65)
    tip_zeta = rng.uniform(0.45, 0.85)

    wbic["Qa_entry"] = clamp(base["wbic"]["Qa_entry"] * rng.uniform(0.85, 3.2), 3.0, 24.0)
    wbic["Qf_entry"] = clamp(base["wbic"]["Qf_entry"] * rng.uniform(0.25, 1.0), 0.05, 0.8)
    wbic["body_ori_task_kp"] = body_ori_kp
    wbic["body_pos_task_kp"] = body_pos_kp
    wbic["tip_pos_task_kp"] = tip_kp
    wbic["body_ori_task_kd"] = 2.0 * body_ori_zeta * math.sqrt(body_ori_kp)
    wbic["body_pos_task_kd"] = 2.0 * body_pos_zeta * math.sqrt(body_pos_kp)
    wbic["tip_pos_task_kd"] = 2.0 * tip_zeta * math.sqrt(tip_kp)
    wbic["joint_kp_stance"] = base["wbic"]["joint_kp_stance"] * rng.uniform(0.9, 1.5)
    wbic["joint_kd_stance"] = base["wbic"]["joint_kd_stance"] * rng.uniform(0.45, 1.0)
    wbic["joint_kp_swing"] = base["wbic"]["joint_kp_swing"] * rng.uniform(0.85, 1.35)
    wbic["joint_kd_swing"] = base["wbic"]["joint_kd_swing"] * rng.uniform(0.45, 1.0)
    params["wbic"] = clamp_wbic_config(wbic)

    swing = params["swing_controller"]
    x_front = clamp(float(np.mean(base["swing_controller"]["interleave_x"][:2])) + rng.uniform(-0.01, 0.015), -0.02, 0.03)
    x_rear = clamp(float(np.mean(base["swing_controller"]["interleave_x"][2:])) + rng.uniform(-0.01, 0.015), -0.02, 0.03)
    y_value = clamp(float(np.mean(base["swing_controller"]["interleave_y"])) * rng.uniform(0.75, 1.35), 0.05, 0.11)
    swing["interleave_x"] = [x_front, x_front, x_rear, x_rear]
    swing["interleave_y"] = [y_value, y_value, y_value, y_value]
    swing["dz_near_ground"] = -clamp(abs(base["swing_controller"]["dz_near_ground"]) * rng.uniform(0.5, 3.0), 0.005, 0.08)
    swing["k1_fsp"] = clamp(base["swing_controller"]["k1_fsp"] * rng.uniform(0.5, 2.8), 0.005, 0.08)
    params["swing_controller"] = clamp_swing_config(swing)

    stance = params["stance_controller_mpc"]
    q_base = base["stance_controller_mpc"]["Q"]
    r_base = base["stance_controller_mpc"]["R"]
    stance["horizon"] = int(clamp(base["stance_controller_mpc"]["horizon"] + rng.randint(-3, 3), 10, 20))
    stance["friction"] = clamp(base["stance_controller_mpc"]["friction"] * rng.uniform(0.8, 1.35), 0.22, 0.5)
    stance["f_min"] = clamp(base["stance_controller_mpc"]["f_min"] * rng.uniform(0.75, 1.25), 2.0, 12.0)
    stance["f_max"] = clamp(base["stance_controller_mpc"]["f_max"] * rng.uniform(0.8, 1.3), 50.0, 120.0)
    stance["Q"] = _scale_q_groups(
        q_base,
        ori_scale=math.exp(rng.uniform(math.log(0.7), math.log(1.4))),
        yaw_scale=math.exp(rng.uniform(math.log(0.7), math.log(1.5))),
        pos_xy_scale=math.exp(rng.uniform(math.log(0.7), math.log(1.5))),
        pos_z_scale=math.exp(rng.uniform(math.log(0.75), math.log(1.45))),
        ang_xy_scale=math.exp(rng.uniform(math.log(0.6), math.log(1.8))),
        yaw_rate_scale=math.exp(rng.uniform(math.log(0.6), math.log(1.8))),
        lin_xy_scale=math.exp(rng.uniform(math.log(0.75), math.log(1.7))),
        lin_z_scale=math.exp(rng.uniform(math.log(0.75), math.log(1.5))),
    )
    stance["R"] = _scale_r_groups(
        r_base,
        xy_scale=math.exp(rng.uniform(math.log(0.7), math.log(1.45))),
        z_scale=math.exp(rng.uniform(math.log(0.7), math.log(1.45))),
    )
    stance["Q_gain"] = clamp(base["stance_controller_mpc"]["Q_gain"] * math.exp(rng.uniform(math.log(0.45), math.log(1.8))), 250.0, 3000.0)
    stance["R_gain"] = clamp(base["stance_controller_mpc"]["R_gain"] * math.exp(rng.uniform(math.log(0.55), math.log(2.8))), 1.5e-4, 3.0e-3)
    params["stance_controller_mpc"] = clamp_stance_config(stance)
    params["gait"] = random_gait_config(rng)

    return normalize_candidate(params)


def build_local_candidate(center: dict[str, Any], rng: random.Random) -> dict[str, Any]:
    params = copy.deepcopy(center)

    wbic = params["wbic"]
    body_ori_kp = clamp(wbic["body_ori_task_kp"] * rng.uniform(0.92, 1.12), 130.0, 420.0)
    body_pos_kp = clamp(wbic["body_pos_task_kp"] * rng.uniform(0.92, 1.12), 120.0, 420.0)
    tip_kp = clamp(wbic["tip_pos_task_kp"] * rng.uniform(0.92, 1.12), 220.0, 700.0)
    body_ori_zeta = clamp(infer_damping_ratio(wbic["body_ori_task_kp"], wbic["body_ori_task_kd"]) * rng.uniform(0.92, 1.08), 0.8, 1.8)
    body_pos_zeta = clamp(infer_damping_ratio(wbic["body_pos_task_kp"], wbic["body_pos_task_kd"]) * rng.uniform(0.92, 1.08), 0.9, 1.9)
    tip_zeta = clamp(infer_damping_ratio(wbic["tip_pos_task_kp"], wbic["tip_pos_task_kd"]) * rng.uniform(0.92, 1.10), 0.35, 1.0)
    wbic["Qa_entry"] *= rng.uniform(0.92, 1.12)
    wbic["Qf_entry"] *= rng.uniform(0.88, 1.12)
    wbic["body_ori_task_kp"] = body_ori_kp
    wbic["body_pos_task_kp"] = body_pos_kp
    wbic["tip_pos_task_kp"] = tip_kp
    wbic["body_ori_task_kd"] = 2.0 * body_ori_zeta * math.sqrt(body_ori_kp)
    wbic["body_pos_task_kd"] = 2.0 * body_pos_zeta * math.sqrt(body_pos_kp)
    wbic["tip_pos_task_kd"] = 2.0 * tip_zeta * math.sqrt(tip_kp)
    wbic["joint_kp_stance"] *= rng.uniform(0.92, 1.10)
    wbic["joint_kd_stance"] *= rng.uniform(0.90, 1.10)
    wbic["joint_kp_swing"] *= rng.uniform(0.92, 1.10)
    wbic["joint_kd_swing"] *= rng.uniform(0.90, 1.10)
    params["wbic"] = clamp_wbic_config(wbic)

    swing = params["swing_controller"]
    x_front = float(np.mean(swing["interleave_x"][:2])) + rng.uniform(-0.005, 0.005)
    x_rear = float(np.mean(swing["interleave_x"][2:])) + rng.uniform(-0.005, 0.005)
    y_value = float(np.mean(swing["interleave_y"])) * rng.uniform(0.94, 1.08)
    swing["interleave_x"] = [x_front, x_front, x_rear, x_rear]
    swing["interleave_y"] = [y_value, y_value, y_value, y_value]
    swing["dz_near_ground"] = -abs(swing["dz_near_ground"]) * rng.uniform(0.88, 1.2)
    swing["k1_fsp"] *= rng.uniform(0.9, 1.18)
    params["swing_controller"] = clamp_swing_config(swing)

    stance = params["stance_controller_mpc"]
    stance["horizon"] = int(clamp(stance["horizon"] + rng.randint(-1, 1), 10, 20))
    stance["friction"] *= rng.uniform(0.94, 1.08)
    stance["f_min"] *= rng.uniform(0.92, 1.08)
    stance["f_max"] *= rng.uniform(0.92, 1.08)
    stance["Q"] = _scale_q_groups(
        stance["Q"],
        ori_scale=rng.uniform(0.94, 1.08),
        yaw_scale=rng.uniform(0.94, 1.08),
        pos_xy_scale=rng.uniform(0.94, 1.08),
        pos_z_scale=rng.uniform(0.94, 1.08),
        ang_xy_scale=rng.uniform(0.92, 1.10),
        yaw_rate_scale=rng.uniform(0.92, 1.10),
        lin_xy_scale=rng.uniform(0.92, 1.10),
        lin_z_scale=rng.uniform(0.94, 1.08),
    )
    stance["R"] = _scale_r_groups(
        stance["R"],
        xy_scale=rng.uniform(0.94, 1.08),
        z_scale=rng.uniform(0.94, 1.08),
    )
    stance["Q_gain"] *= rng.uniform(0.88, 1.18)
    stance["R_gain"] *= rng.uniform(0.88, 1.18)
    params["stance_controller_mpc"] = clamp_stance_config(stance)
    params["gait"] = local_gait_config(params["gait"], rng)

    return normalize_candidate(params)


def curated_candidates(base: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    candidates: list[tuple[str, dict[str, Any]]] = []

    tracking = copy.deepcopy(base)
    tracking["wbic"].update(
        {
            "Qf_entry": 0.45,
            "Qa_entry": 5.0,
            "body_ori_task_kp": 180.0,
            "body_ori_task_kd": 40.0,
            "body_pos_task_kp": 170.0,
            "body_pos_task_kd": 42.0,
            "tip_pos_task_kp": 310.0,
            "tip_pos_task_kd": 20.0,
            "joint_kp_stance": 19.0,
            "joint_kd_stance": 0.65,
            "joint_kp_swing": 16.0,
            "joint_kd_swing": 0.55,
        }
    )
    tracking["swing_controller"].update(
        {
            "interleave_x": [0.004, 0.004, 0.0, 0.0],
            "interleave_y": [0.082, 0.082, 0.082, 0.082],
            "dz_near_ground": -0.025,
            "k1_fsp": 0.024,
        }
    )
    tracking["stance_controller_mpc"].update(
        {
            "horizon": 14,
            "friction": 0.32,
            "f_min": 5.0,
            "f_max": 82.0,
            "Q": _scale_q_groups(
                base["stance_controller_mpc"]["Q"],
                ori_scale=1.05,
                yaw_scale=1.05,
                pos_xy_scale=1.0,
                pos_z_scale=1.05,
                ang_xy_scale=1.0,
                yaw_rate_scale=1.0,
                lin_xy_scale=1.15,
                lin_z_scale=1.0,
            ),
            "R": _scale_r_groups(base["stance_controller_mpc"]["R"], xy_scale=1.0, z_scale=1.0),
            "Q_gain": 1200.0,
            "R_gain": 6.5e-4,
        }
    )
    tracking["gait"] = gait_from_library("walk_diag_a", WALK_DIAG_A, 0.60, 0.30, 0.12)
    candidates.append(("seed_tracking", normalize_candidate(tracking)))

    clearance = copy.deepcopy(base)
    clearance["wbic"].update(
        {
            "Qf_entry": 0.55,
            "Qa_entry": 4.2,
            "body_ori_task_kp": 165.0,
            "body_ori_task_kd": 38.0,
            "body_pos_task_kp": 155.0,
            "body_pos_task_kd": 40.0,
            "tip_pos_task_kp": 285.0,
            "tip_pos_task_kd": 18.5,
            "joint_kp_stance": 18.0,
            "joint_kd_stance": 0.68,
            "joint_kp_swing": 15.0,
            "joint_kd_swing": 0.58,
        }
    )
    clearance["swing_controller"].update(
        {
            "interleave_x": [0.0, 0.0, 0.0, 0.0],
            "interleave_y": [0.078, 0.078, 0.078, 0.078],
            "dz_near_ground": -0.035,
            "k1_fsp": 0.018,
        }
    )
    clearance["stance_controller_mpc"].update(
        {
            "horizon": 15,
            "friction": 0.3,
            "f_min": 4.5,
            "f_max": 78.0,
            "Q": _scale_q_groups(
                base["stance_controller_mpc"]["Q"],
                ori_scale=1.0,
                yaw_scale=1.0,
                pos_xy_scale=0.95,
                pos_z_scale=1.1,
                ang_xy_scale=1.0,
                yaw_rate_scale=1.0,
                lin_xy_scale=1.05,
                lin_z_scale=1.0,
            ),
            "R": _scale_r_groups(base["stance_controller_mpc"]["R"], xy_scale=1.05, z_scale=1.05),
            "Q_gain": 900.0,
            "R_gain": 8.0e-4,
        }
    )
    clearance["gait"] = gait_from_library("walk_diag_b", WALK_DIAG_B, 0.66, 0.32, 0.14)
    candidates.append(("seed_clearance", normalize_candidate(clearance)))

    agile = copy.deepcopy(base)
    agile["wbic"].update(
        {
            "Qf_entry": 0.5,
            "Qa_entry": 4.5,
            "body_ori_task_kp": 175.0,
            "body_ori_task_kd": 39.0,
            "body_pos_task_kp": 165.0,
            "body_pos_task_kd": 41.0,
            "tip_pos_task_kp": 320.0,
            "tip_pos_task_kd": 19.5,
            "joint_kp_stance": 20.0,
            "joint_kd_stance": 0.62,
            "joint_kp_swing": 16.5,
            "joint_kd_swing": 0.54,
        }
    )
    agile["swing_controller"].update(
        {
            "interleave_x": [0.006, 0.006, 0.004, 0.004],
            "interleave_y": [0.086, 0.086, 0.086, 0.086],
            "dz_near_ground": -0.022,
            "k1_fsp": 0.03,
        }
    )
    agile["stance_controller_mpc"].update(
        {
            "horizon": 16,
            "friction": 0.34,
            "f_min": 5.0,
            "f_max": 85.0,
            "Q": _scale_q_groups(
                base["stance_controller_mpc"]["Q"],
                ori_scale=1.08,
                yaw_scale=1.12,
                pos_xy_scale=1.0,
                pos_z_scale=1.0,
                ang_xy_scale=1.05,
                yaw_rate_scale=1.1,
                lin_xy_scale=1.2,
                lin_z_scale=1.0,
            ),
            "R": _scale_r_groups(base["stance_controller_mpc"]["R"], xy_scale=0.95, z_scale=1.0),
            "Q_gain": 1300.0,
            "R_gain": 6.0e-4,
        }
    )
    agile["gait"] = gait_from_library("trot", TROT, 0.38, 0.24, 0.11)
    candidates.append(("seed_agile", normalize_candidate(agile)))

    return candidates


def failed_result(
    tag: str,
    params: dict[str, Any],
    unstable_reason: str,
    roll_pitch_peak: float = 0.0,
    base_z_min: float = 0.0,
) -> CandidateResult:
    return CandidateResult(
        tag=tag,
        params=params,
        stable=False,
        score=2000.0,
        metrics={
            "body_pos_rmse": 0.0,
            "body_ori_rmse": 0.0,
            "body_vel_rmse": 0.0,
            "foot_rmse": 0.0,
            "stance_foot_rmse": 0.0,
            "swing_foot_rmse": 0.0,
            "torque_mean": 0.0,
            "torque_peak": 0.0,
            "sat_fraction": 0.0,
            "roll_pitch_peak": roll_pitch_peak,
            "base_z_min": base_z_min,
            "sample_count": 0.0,
            "unstable_reason": unstable_reason,
        },
    )


def evaluate_candidate(
    tag: str,
    params: dict[str, Any],
    simulator: ManagedSimulator,
    publisher: CommandPublisher,
    telemetry: TelemetrySubscriber,
    scenario: ValidationScenario,
) -> CandidateResult:
    tmpdir = workspace_for_candidate(params)
    tmp_path = Path(tmpdir.name)
    log_dir = tmp_path / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    runner = ControllerRunner(tmp_path / "config", LCM_DEFAULT_URL, log_dir)
    launch_ts = time.monotonic()
    simulator.reset()

    pos_sq = 0.0
    ori_sq = 0.0
    vel_sq = 0.0
    foot_sq = 0.0
    swing_foot_sq = 0.0
    stance_foot_xy_sq = 0.0
    torque_sum = 0.0
    sat_sum = 0.0
    torque_peak = 0.0
    roll_pitch_peak = 0.0
    base_z_min = math.inf
    base_x_max = -math.inf
    base_z_max = -math.inf
    final_base_x = 0.0
    final_base_z = 0.0
    sample_count = 0
    swing_sample_count = 0
    stance_sample_count = 0
    stable = True
    unstable_reason = ""

    try:
        controllers_ready = False
        boot_deadline = time.monotonic() + 1.5
        while time.monotonic() < boot_deadline:
            if not runner.running():
                stable = False
                unstable_reason = "controller_exit"
                break
            publisher.publish(warmup_command())
            simulator.step()
            _, _, wbc_ts, _ = telemetry.snapshot()
            if wbc_ts >= launch_ts:
                controllers_ready = True
                break

        if stable and not controllers_ready:
            stable = False
            unstable_reason = "no_wbc_boot"

        warmup_start = time.monotonic()
        while time.monotonic() - warmup_start < WARMUP_DURATION:
            if not runner.running():
                stable = False
                unstable_reason = "controller_exit"
                break
            publisher.publish(warmup_command())
            simulator.step()

        if not stable:
            return failed_result(tag, params, unstable_reason)

        start = time.monotonic()
        while time.monotonic() - start < scenario.duration:
            if not runner.running():
                stable = False
                unstable_reason = "controller_exit"
                break

            command = scenario.schedule_fn(time.monotonic() - start, params)
            publisher.publish(command)
            simulator.step()

            wbc_cmd, phase_msg, wbc_ts, phase_ts = telemetry.snapshot()

            base_pos = np.asarray(simulator.sim.body_lin_pos, dtype=float)
            base_euler = np.asarray(simulator.sim.imu_data[10:13], dtype=float)
            base_vel = np.asarray(simulator.sim.body_lin_vel, dtype=float)
            base_quat = np.asarray(simulator.sim.imu_data[3:7], dtype=float)

            if (
                not np.isfinite(base_pos).all()
                or not np.isfinite(base_euler).all()
                or not np.isfinite(base_vel).all()
                or not np.isfinite(base_quat).all()
            ):
                stable = False
                unstable_reason = "nan_state"
                break

            base_z_min = min(base_z_min, float(base_pos[2]))
            base_x_max = max(base_x_max, float(base_pos[0]))
            base_z_max = max(base_z_max, float(base_pos[2]))
            final_base_x = float(base_pos[0])
            final_base_z = float(base_pos[2])
            roll_pitch_peak = max(
                roll_pitch_peak,
                float(abs(base_euler[0])),
                float(abs(base_euler[1])),
            )

            if base_pos[2] < 0.10 or abs(base_euler[0]) > 0.85 or abs(base_euler[1]) > 0.85:
                stable = False
                unstable_reason = "fall"
                break

            now_ts = time.monotonic()
            if wbc_cmd is None or (now_ts - wbc_ts) > 0.4:
                continue

            des_pos = np.asarray(wbc_cmd.body.position, dtype=float)
            des_vel = np.asarray(wbc_cmd.body.lin_vel, dtype=float)
            des_quat = np.asarray(wbc_cmd.body.orientation_quaternion, dtype=float)
            des_feet = np.asarray(
                [
                    wbc_cmd.legs.r1_pos,
                    wbc_cmd.legs.l1_pos,
                    wbc_cmd.legs.r2_pos,
                    wbc_cmd.legs.l2_pos,
                ],
                dtype=float,
            )

            act_feet = simulator.foot_positions_global()
            actuation = np.abs(simulator.actuation()) / TAU_LIMITS

            pos_sq += float(np.sum((base_pos - des_pos) ** 2))
            ori_sq += quat_angle(base_quat, des_quat) ** 2
            vel_sq += float(np.sum((base_vel - des_vel) ** 2))
            foot_errors = act_feet - des_feet
            foot_sq += float(np.mean(np.sum(foot_errors**2, axis=1)))
            torque_sum += float(np.mean(actuation))
            sat_sum += float(np.mean(actuation > 0.95))
            torque_peak = max(torque_peak, float(np.max(actuation)))
            sample_count += 1

            phase_fresh = phase_msg is not None and (now_ts - phase_ts) <= 0.4
            if phase_fresh:
                for leg_idx, phase in enumerate(phase_msg.phase):
                    leg_error = foot_errors[leg_idx]
                    if int(phase) in SWING_PHASES:
                        swing_foot_sq += float(np.dot(leg_error, leg_error))
                        swing_sample_count += 1
                    elif int(phase) in STANCE_PHASES:
                        stance_foot_xy_sq += float(np.dot(leg_error[:2], leg_error[:2]))
                        stance_sample_count += 1

        if sample_count == 0:
            stable = False
            unstable_reason = unstable_reason or "no_reference_data"

        body_pos_rmse = math.sqrt(pos_sq / max(sample_count, 1) / 3.0)
        body_ori_rmse = math.sqrt(ori_sq / max(sample_count, 1))
        body_vel_rmse = math.sqrt(vel_sq / max(sample_count, 1) / 3.0)
        foot_rmse = math.sqrt(foot_sq / max(sample_count, 1))
        if swing_sample_count > 0:
            swing_foot_rmse = math.sqrt(swing_foot_sq / swing_sample_count)
        else:
            swing_foot_rmse = foot_rmse
        if stance_sample_count > 0:
            stance_foot_rmse = math.sqrt(stance_foot_xy_sq / stance_sample_count)
        else:
            stance_foot_rmse = foot_rmse
        torque_mean = torque_sum / max(sample_count, 1)
        sat_fraction = sat_sum / max(sample_count, 1)
        base_z_min_value = 0.0 if math.isinf(base_z_min) else base_z_min
        base_x_max_value = 0.0 if math.isinf(base_x_max) else base_x_max
        base_z_max_value = 0.0 if math.isinf(base_z_max) else base_z_max
        stairs_front_penalty = max(0.0, STAIRS_PROFILE.front_x - base_x_max_value)
        stairs_progress_penalty = max(0.0, STAIRS_TARGET_X - final_base_x)
        stairs_height_penalty = max(0.0, STAIRS_TARGET_BODY_Z - final_base_z)

        score = (
            130.0 * body_pos_rmse
            + 95.0 * body_ori_rmse
            + 35.0 * body_vel_rmse
            + 28.0 * stance_foot_rmse
            + 42.0 * swing_foot_rmse
            + 6.0 * torque_mean
            + 24.0 * sat_fraction
            + 12.0 * roll_pitch_peak
            + 8.0 * max(0.0, torque_peak - 0.85)
            + 80.0 * max(0.0, 0.18 - base_z_min_value)
            + 30.0 * stairs_front_penalty
            + 55.0 * stairs_progress_penalty
            + 80.0 * stairs_height_penalty
        )

        if not stable:
            expected_samples = scenario.duration * 400.0
            missing_samples = max(0.0, expected_samples - float(sample_count))
            score += 1000.0 + 0.25 * missing_samples

        metrics: dict[str, Any] = {
            "body_pos_rmse": body_pos_rmse,
            "body_ori_rmse": body_ori_rmse,
            "body_vel_rmse": body_vel_rmse,
            "foot_rmse": foot_rmse,
            "stance_foot_rmse": stance_foot_rmse,
            "swing_foot_rmse": swing_foot_rmse,
            "torque_mean": torque_mean,
            "torque_peak": torque_peak,
            "sat_fraction": sat_fraction,
            "roll_pitch_peak": roll_pitch_peak,
            "base_z_min": base_z_min_value,
            "base_x_final": final_base_x,
            "base_x_max": base_x_max_value,
            "base_z_final": final_base_z,
            "base_z_max": base_z_max_value,
            "stairs_front_penalty": stairs_front_penalty,
            "stairs_progress_penalty": stairs_progress_penalty,
            "stairs_height_penalty": stairs_height_penalty,
            "sample_count": float(sample_count),
            "scenario": scenario.name,
        }
        if unstable_reason:
            metrics["unstable_reason"] = unstable_reason

        return CandidateResult(
            tag=tag,
            params=params,
            stable=stable,
            score=score,
            metrics=metrics,
        )
    finally:
        publisher.publish(disabled_command())
        runner.stop()
        tmpdir.cleanup()


def aggregate_results(tag: str, params: dict[str, Any], results: list[CandidateResult]) -> CandidateResult:
    stable = all(result.stable for result in results)
    score = float(sum(result.score for result in results) / max(len(results), 1))
    avg_keys = [
        "body_pos_rmse",
        "body_ori_rmse",
        "body_vel_rmse",
        "foot_rmse",
        "stance_foot_rmse",
        "swing_foot_rmse",
        "torque_mean",
        "sat_fraction",
        "base_x_final",
        "base_z_final",
        "stairs_front_penalty",
        "stairs_progress_penalty",
        "stairs_height_penalty",
    ]
    metrics: dict[str, Any] = {
        key: float(sum(float(result.metrics[key]) for result in results) / len(results))
        for key in avg_keys
    }
    metrics["torque_peak"] = float(max(float(result.metrics["torque_peak"]) for result in results))
    metrics["roll_pitch_peak"] = float(max(float(result.metrics["roll_pitch_peak"]) for result in results))
    metrics["base_z_min"] = float(min(float(result.metrics["base_z_min"]) for result in results))
    metrics["base_x_max"] = float(max(float(result.metrics["base_x_max"]) for result in results))
    metrics["base_z_max"] = float(max(float(result.metrics["base_z_max"]) for result in results))
    metrics["sample_count"] = float(sum(float(result.metrics["sample_count"]) for result in results))
    metrics["scenario_count"] = len(results)

    unstable_scenarios = [
        result.metrics["scenario"]
        for result in results
        if not result.stable and "scenario" in result.metrics
    ]
    if unstable_scenarios:
        metrics["unstable_scenarios"] = unstable_scenarios

    details = [
        {
            "tag": result.tag,
            "stable": result.stable,
            "score": result.score,
            "metrics": result.metrics,
        }
        for result in results
    ]
    return CandidateResult(
        tag=tag,
        params=params,
        stable=stable,
        score=score,
        metrics=metrics,
        details=details,
    )


def evaluate_scenarios(
    tag: str,
    params: dict[str, Any],
    simulator: ManagedSimulator,
    publisher: CommandPublisher,
    telemetry: TelemetrySubscriber,
    scenarios: list[ValidationScenario],
) -> CandidateResult:
    results = [
        evaluate_candidate(
            f"{tag}:{scenario.name}",
            params,
            simulator,
            publisher,
            telemetry,
            scenario,
        )
        for scenario in scenarios
    ]
    if len(results) == 1:
        single = results[0]
        return CandidateResult(
            tag=tag,
            params=params,
            stable=single.stable,
            score=single.score,
            metrics=single.metrics,
            details=[
                {
                    "tag": single.tag,
                    "stable": single.stable,
                    "score": single.score,
                    "metrics": single.metrics,
                }
            ],
        )
    return aggregate_results(tag, params, results)


def result_sort_key(result: CandidateResult) -> tuple[int, float]:
    return (0 if result.stable else 1, result.score)


def print_result(result: CandidateResult) -> None:
    metrics = result.metrics
    gait = result.params.get("gait", {})
    gait_name = gait.get("name", "gait")
    print(
        f"{result.tag:>16} | stable={str(result.stable):<5} | score={result.score:8.3f} | "
        f"{gait_name}:{gait.get('t_st', 0.0):.2f}/{gait.get('t_sw', 0.0):.2f}/{gait.get('stride_height', 0.0):.2f} | "
        f"body={metrics['body_pos_rmse']:.4f}/{metrics['body_ori_rmse']:.4f} | "
        f"feet={metrics['stance_foot_rmse']:.4f}/{metrics['swing_foot_rmse']:.4f} | "
        f"xz={metrics['base_x_final']:.2f}/{metrics['base_z_final']:.2f} | "
        f"torque={metrics['torque_mean']:.3f}/{metrics['torque_peak']:.3f} | "
        f"sat={metrics['sat_fraction']:.3f} | peak_rp={metrics['roll_pitch_peak']:.3f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Simulation-based tuning for WBIC, swing and stance MPC configs")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--search", type=int, default=10, help="number of random candidates")
    parser.add_argument("--local", type=int, default=6, help="number of local refinements")
    parser.add_argument("--finalists", type=int, default=4)
    parser.add_argument("--short-duration", type=float, default=6.2)
    parser.add_argument("--final-duration", type=float, default=12.2)
    parser.add_argument("--freeze-swing", action="store_true", help="keep swing_controller.yaml fixed at the current config values")
    parser.add_argument("--lock-f-max", action="store_true", help="keep stance_controller_mpc.yaml:f_max fixed at the current config value")
    parser.add_argument("--output-json", type=Path, default=None)
    args = parser.parse_args()

    os.environ["LCM_DEFAULT_URL"] = LCM_DEFAULT_URL

    base = load_base_params()
    rng = random.Random(args.seed)
    simulator = ManagedSimulator()
    telemetry = TelemetrySubscriber(LCM_DEFAULT_URL)
    publisher = CommandPublisher(LCM_DEFAULT_URL)

    short_scenarios = [ValidationScenario("screen", short_schedule, args.short_duration)]
    final_scenarios = [
        ValidationScenario("stairs_nominal", final_schedule, args.final_duration),
        ValidationScenario("stairs_margin", margin_schedule, args.final_duration),
    ]

    def constrained(params: dict[str, Any]) -> dict[str, Any]:
        return apply_search_constraints(
            params,
            base,
            freeze_swing=args.freeze_swing,
            lock_f_max=args.lock_f_max,
        )

    try:
        all_short_results: list[CandidateResult] = []

        baseline = evaluate_scenarios(
            "baseline",
            constrained(base),
            simulator,
            publisher,
            telemetry,
            short_scenarios,
        )
        print_result(baseline)
        all_short_results.append(baseline)

        for tag, candidate in curated_candidates(base):
            result = evaluate_scenarios(
                tag,
                constrained(candidate),
                simulator,
                publisher,
                telemetry,
                short_scenarios,
            )
            print_result(result)
            all_short_results.append(result)

        for idx in range(args.search):
            candidate = constrained(build_candidate(base, rng))
            result = evaluate_scenarios(
                f"search_{idx + 1}",
                candidate,
                simulator,
                publisher,
                telemetry,
                short_scenarios,
            )
            print_result(result)
            all_short_results.append(result)

        best_short = min(all_short_results, key=result_sort_key)
        for idx in range(args.local):
            candidate = constrained(build_local_candidate(best_short.params, rng))
            result = evaluate_scenarios(
                f"local_{idx + 1}",
                candidate,
                simulator,
                publisher,
                telemetry,
                short_scenarios,
            )
            print_result(result)
            all_short_results.append(result)

        finalists = sorted(all_short_results, key=result_sort_key)[: args.finalists]
        if all(item.tag != baseline.tag for item in finalists):
            finalists.append(baseline)

        print("\nFinal validation:")
        final_results: list[CandidateResult] = []
        for result in finalists:
            validated = evaluate_scenarios(
                f"final_{result.tag}",
                result.params,
                simulator,
                publisher,
                telemetry,
                final_scenarios,
            )
            print_result(validated)
            final_results.append(validated)

        best_final = min(final_results, key=result_sort_key)

        payload = {
            "seed": args.seed,
            "freeze_swing": args.freeze_swing,
            "lock_f_max": args.lock_f_max,
            "short_scenarios": [
                {"name": scenario.name, "duration": scenario.duration}
                for scenario in short_scenarios
            ],
            "final_scenarios": [
                {"name": scenario.name, "duration": scenario.duration}
                for scenario in final_scenarios
            ],
            "best": {
                "tag": best_final.tag,
                "stable": best_final.stable,
                "score": best_final.score,
                "metrics": best_final.metrics,
                "params": best_final.params,
                "details": best_final.details,
            },
            "short_results": [
                {
                    "tag": item.tag,
                    "stable": item.stable,
                    "score": item.score,
                    "metrics": item.metrics,
                    "params": item.params,
                    "details": item.details,
                }
                for item in all_short_results
            ],
            "final_results": [
                {
                    "tag": item.tag,
                    "stable": item.stable,
                    "score": item.score,
                    "metrics": item.metrics,
                    "params": item.params,
                    "details": item.details,
                }
                for item in final_results
            ],
        }

        print("\nBest parameters:")
        print(json.dumps(payload["best"], indent=2))

        if args.output_json is not None:
            args.output_json.parent.mkdir(parents=True, exist_ok=True)
            args.output_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

        return 0
    finally:
        telemetry.close()
        try:
            simulator.sim.env.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
