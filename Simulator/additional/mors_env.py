"""
MuJoCo implementation of MORS Gym environment.
"""

import os
import inspect
import time
import math
import threading
import numpy as np
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation
from transforms3d.quaternions import quat2mat

from additional.mjcf import load_mjmodel
from additional.motor_accurate import MotorModel


NUM_SUBSTEPS = 2
NUM_MOTORS = 12

MOTOR_NAMES = [
    "abad_joint_R1", "hip_joint_R1", "knee_joint_R1",
    "abad_joint_L1", "hip_joint_L1", "knee_joint_L1",
    "abad_joint_R2", "hip_joint_R2", "knee_joint_R2",
    "abad_joint_L2", "hip_joint_L2", "knee_joint_L2",
]

MOTOR_VELOCITY_MAX = [28, 28, 14]
MOTOR_TORQUE_MAX = [6.0, 6.0, 12.0]
BASE_ORIENTATION_MAX = 1.57

DEPTH_CAMERA_NAME = "depth_camera"
D435I_DEPTH_FOVY_DEG = 58.0
D435I_DEPTH_RANGE_M = (0.105, 3.0)

class MorsMujocoEnv():

    def __init__(self,
                 xml_path="../MJCF/mors.xml",
                 render_quality="high",
                 scene=None,
                 sim_freq=500,
                 motor_kp=30.0,
                 motor_kd=0.1,
                 ext_disturbance_enabled=False,
                 depth_image_enabled=False,
                 depth_image_size=(424, 240),
                 init_motor_angles=[ 0.0, -1.57,  3.14,
                                    -0.0,  1.57, -3.14,
                                    -0.0, -1.57,  3.14,
                                     0.0,  1.57, -3.14]):
        
        self._motor_kp = motor_kp
        self._motor_kd = motor_kd
        self._xml_path = xml_path
        self._render_quality = str(render_quality).strip().lower()
        self._scene = scene
        self._sim_freq = sim_freq
        self._ext_disturbance_enabled = ext_disturbance_enabled
        self._time_step = 1 / sim_freq
        self._env_step_counter = 0
        self._sim_data_lock = threading.Lock()
        self._depth_renderer_lock = threading.Lock()

        self._depth_enabled = bool(depth_image_enabled)
        self._depth_width = int(depth_image_size[0])
        self._depth_height = int(depth_image_size[1])
        self._depth_camera_name = DEPTH_CAMERA_NAME
        self._depth_camera_id = -1
        self._depth_renderer = None
        self._depth_data = None
        self._depth_min_range_m = D435I_DEPTH_RANGE_M[0]
        self._depth_max_range_m = D435I_DEPTH_RANGE_M[1]

        # ---------------- MuJoCo ----------------
        self.model = load_mjmodel(xml_path, scene=scene)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self._time_step
        self.viewer = mujoco.viewer.launch_passive(self.model, 
                                                   self.data,
                                                   show_left_ui=False,
                                                   show_right_ui=False)
        self.data.qpos[7:7+NUM_MOTORS] = init_motor_angles[:]
        mujoco.mj_forward(self.model, self.data)
        self._apply_render_quality()
        if self._depth_enabled:
            self._init_depth_camera()

        # motor id mapping
        self._motor_id_list = [mujoco.mj_name2id(self.model,
                                                 mujoco.mjtObj.mjOBJ_JOINT,
                                                 name)
                               for name in MOTOR_NAMES]
        
        # --------------- Init motors ---------------
        self._motor_model = MotorModel(NUM_MOTORS)
        self._motor_model.set_kp(self._motor_kp)
        self._motor_model.set_kd(self._motor_kd)

        # --------------- External disturbance params ---------------
        self._force_dir = 1
        self._force_arrow_length = 0.6
        self._force_arrow_width = 0.01

    def _init_depth_camera(self):
        self._depth_camera_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            self._depth_camera_name,
        )
        if self._depth_camera_id < 0:
            raise ValueError(
                f"Depth camera `{self._depth_camera_name}` is missing in MJCF `{self._xml_path}`"
            )

        self.model.cam_fovy[self._depth_camera_id] = D435I_DEPTH_FOVY_DEG

    def _ensure_depth_renderer(self):
        if self._depth_renderer is not None:
            return

        # OpenGL context must be created and used from the same (depth) thread.
        self._depth_renderer = mujoco.Renderer(
            self.model,
            width=self._depth_width,
            height=self._depth_height,
        )
        self._depth_data = mujoco.MjData(self.model)

    def _iter_render_scenes(self):
        scenes = []

        for attr_name in ("user_scn", "scn", "_scn"):
            scene = getattr(self.viewer, attr_name, None)
            if scene is not None:
                scenes.append(scene)

        get_sim = getattr(self.viewer, "_get_sim", None)
        if callable(get_sim):
            sim = get_sim()
            if sim is not None:
                for attr_name in ("user_scn", "scn", "_scn"):
                    scene = getattr(sim, attr_name, None)
                    if scene is not None:
                        scenes.append(scene)

        unique_scenes = []
        seen_ids = set()
        for scene in scenes:
            scene_id = id(scene)
            if scene_id in seen_ids:
                continue
            seen_ids.add(scene_id)
            unique_scenes.append(scene)

        return unique_scenes

    def _apply_render_quality(self):
        if self.viewer is None:
            return

        render_features_enabled = self._render_quality != "low"
        render_flag_names = (
            "mjRND_SHADOW",
            "mjRND_REFLECTION",
            "mjRND_SKYBOX",
            "mjRND_HAZE",
            "mjRND_CULL_FACE",
        )

        with self.viewer.lock():
            for scene in self._iter_render_scenes():
                flags = getattr(scene, "flags", None)
                if flags is None:
                    continue

                for flag_name in render_flag_names:
                    render_flag = getattr(mujoco.mjtRndFlag, flag_name, None)
                    if render_flag is None:
                        continue
                    flags[render_flag] = int(render_features_enabled)

        self.viewer.sync()

    def _clear_force_visualization(self):
        if self.viewer is None:
            return

        with self.viewer.lock():
            scn = self.viewer.user_scn
            if scn is not None:
                scn.ngeom = 0

    

      
    def step(self, 
             ref_joint_angle : list[float], 
             ref_joint_velocity : list[float], 
             ref_joint_torque : list[float]):
        if len(ref_joint_angle) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} motor angles, got {len(ref_joint_angle)}")
        if len(ref_joint_velocity) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} motor velocities, got {len(ref_joint_velocity)}")
        if len(ref_joint_torque) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} motor torques, got {len(ref_joint_torque)}")
        self._env_step_counter += 1

        with self._sim_data_lock:
            if self._ext_disturbance_enabled:
                self._apply_force()

            self.__apply_action(ref_joint_angle, ref_joint_velocity, ref_joint_torque)
            mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
        
    def __apply_action(self, ref_joint_angle, ref_joint_velocity, ref_joint_torque):

        motor_angles = self.get_motor_angles()
        motor_velocities = self.get_motor_velocities()

        self._motor_model.set_sensor_data(motor_angles, motor_velocities)
        self._motor_model.set_ref_angle(ref_joint_angle)
        self._motor_model.set_ref_vel(ref_joint_velocity)
        self._motor_model.set_ref_torque(ref_joint_torque)
        self._motor_model.set_kp(self._motor_kp)
        self._motor_model.set_kd(self._motor_kd)
        torque = self._motor_model.step()

        for i, joint_id in enumerate(self._motor_id_list):
            self.data.ctrl[i] = torque[i]

    def close(self):
        with self._depth_renderer_lock:
            if self._depth_renderer is not None:
                close_depth_renderer = getattr(self._depth_renderer, "close", None)
                if callable(close_depth_renderer):
                    close_depth_renderer()
                self._depth_renderer = None
                self._depth_data = None
        self.viewer.close()

    def set_kpkd(self, kp, kd):
        self._motor_kp = kp
        self._motor_kd = kd

    # ==========================================================
    # SENSOR ACCESS
    # ==========================================================

    def get_motor_angles(self):
        return self.data.qpos[7:7+NUM_MOTORS].copy()

    def get_motor_velocities(self):
        return self.data.qvel[6:6+NUM_MOTORS].copy()

    def get_motor_torques(self):
        return self.data.qfrc_actuator[:NUM_MOTORS].copy()

    def get_base_position(self):
        pos = self.data.qpos[0:3]
        return pos.copy()

    def get_base_orientation(self):
        quat = self.data.qpos[3:7].copy()
        # normalization
        w, x, y, z = quat
        n = math.sqrt(w*w + x*x + y*y + z*z)
        w, x, y, z = w/n, x/n, y/n, z/n
        # set in order x y z w
        quat = [x, y, z, w]

        return quat

    def get_base_orientation_euler(self):
        quat = self.get_base_orientation()
        euler = Rotation.from_quat(quat, scalar_first=False).as_euler('xyz', degrees=False)
        euler_correct = np.array([euler[0], euler[1], euler[2]])

        return euler_correct.copy()

    def get_base_lin_vel(self):
        return self.data.qvel[0:3].copy()
    
    def get_base_lin_acc(self):
        return self.data.sensor('accelerometer').data.copy()

    def get_base_ang_vel(self):
        ang_vel = self.data.qvel[3:6].copy()
        return ang_vel.copy()
    
    def get_contact_flags(self):
        """
        Returns:
            contact_flags: [R1, L1, R2, L2] boolean flags
        """
        contact_flags = [False] * 4

        foot_body_names = ["ef_R1", "ef_L1", "ef_R2", "ef_L2"]
        foot_body_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in foot_body_names
        ]

        for i in range(self.data.ncon):
            c = self.data.contact[i]
            body1 = self.model.geom_bodyid[c.geom1]
            body2 = self.model.geom_bodyid[c.geom2]

            for leg_index, foot_body_id in enumerate(foot_body_ids):
                if body1 == foot_body_id or body2 == foot_body_id:
                    contact_flags[leg_index] = True

        return contact_flags
    
    def get_global_foot_positions(self):
        """
        Args:
            None
        Returns:
            foot_pos: [x, y, z] position of the foot in world frame
        """
        foot_body_names = ["ef_R1", "ef_L1", "ef_R2", "ef_L2"]
        foot_pos = [0]*4

        for leg_index in range(4):
            foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, foot_body_names[leg_index])
            foot_pos[leg_index] = self.data.body(foot_body_id).xpos.copy()
            
        return foot_pos
    
    def get_local_foot_positions(self, yaw_real):
        foot_body_names = ["ef_R1", "ef_L1", "ef_R2", "ef_L2"]
        foot_pos = [0]*4

        base_pos = np.array(self.get_base_position())
        base_euler = self.get_base_orientation_euler()
        euler_correct = np.array([base_euler[0], base_euler[1], yaw_real])

        R_base = Rotation.from_euler('xyz', euler_correct).as_matrix()

        for leg_index in range(4):
            foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, foot_body_names[leg_index])
            foot_pos[leg_index] = R_base.T @ (self.data.body(foot_body_id).xpos.copy() - base_pos)
            
        return foot_pos

    def _body_lin_vel_world(self, body_id):
        """Линейная скорость начала СК тела в мировых осях.

        data.body(id).cvel НЕ подходит: его layout - (rot:lin), т.е. cvel[:3] это угловая
        скорость, а cvel[3:6] - линейная скорость точки, совпадающей с центром масс поддерева.
        mj_objectVelocity с flg_local=0 возвращает [омега; v] для xpos тела в мировых осях -
        это то, что парно к get_global_foot_positions(), которая берёт именно xpos.
        """
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                 body_id, vel6, 0)
        return vel6[3:6].copy()

    def get_global_foot_velocities(self):
        foot_body_names = ["ef_R1", "ef_L1", "ef_R2", "ef_L2"]
        foot_vel = [0]*4

        for leg_index in range(4):
            foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, foot_body_names[leg_index])
            foot_vel[leg_index] = self._body_lin_vel_world(foot_body_id)
            
        return foot_vel
    
    def get_local_foot_velocities(self, yaw_real):
        foot_body_names = ["ef_R1", "ef_L1", "ef_R2", "ef_L2"]
        foot_vel_global = [0]*4
        foot_vel_local = [0]*4
        foot_pos_global = self.get_global_foot_positions()

        base_pos = np.array(self.get_base_position())
        base_vel = self.get_base_lin_vel()
        base_euler = self.get_base_orientation_euler()
        euler_correct = np.array([base_euler[0], base_euler[1], yaw_real])
        R_base = Rotation.from_euler('xyz', euler_correct).as_matrix()

        # qvel[3:6] свободного джойнта - угловая скорость в СВЯЗАННЫХ осях, а base_vel и
        # скорости стоп - в мировых. Переводим омегу в мировые оси истинной ориентацией
        # (R_base построена по скорректированному yaw и для этого не годится).
        R_true = Rotation.from_quat(self.get_base_orientation()).as_matrix()
        omega_world = R_true @ self.get_base_ang_vel()

        for leg_index in range(4):
            foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, foot_body_names[leg_index])
            foot_vel_global[leg_index] = self._body_lin_vel_world(foot_body_id)

            # Плечо переносной скорости - вектор база->стопа, а не радиус-вектор стопы
            # от начала мировой СК.
            v_transfer = np.cross(omega_world, foot_pos_global[leg_index] - base_pos)
            foot_vel_local[leg_index] = R_base.T @ (foot_vel_global[leg_index] - base_vel - v_transfer)
        
        return foot_vel_local

    # ==========================================================
    # EXTERNAL DISTURBANCES
    # ==========================================================

    def set_ext_forces_params(self, magn, duration, interval): 
        self._ext_force_magn = magn
        self._ext_force_duration = int(duration / self._time_step)
        self._ext_force_interval = int(interval / self._time_step)

    def _draw_force_visualization(self, point, force):
        if self.viewer is None:
            return

        force_norm = np.linalg.norm(force)
        with self.viewer.lock():
            scn = self.viewer.user_scn
            if scn is None:
                return

            scn.ngeom = 0
            if force_norm < 1e-9:
                return

            geom = scn.geoms[0]
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_ARROW,
                np.array([0.01, 0.01, 0.01]),
                np.zeros(3),
                np.eye(3).reshape(-1),
                np.array([1.0, 0.2, 0.2, 0.9], dtype=np.float32),
            )
            geom.category = mujoco.mjtCatBit.mjCAT_DECOR
            geom.objtype = mujoco.mjtObj.mjOBJ_UNKNOWN
            geom.objid = -1
            geom.segid = -1
            geom.emission = 1.0
            tip = point + (force / force_norm) * self._force_arrow_length
            mujoco.mjv_connector(
                geom,
                mujoco.mjtGeom.mjGEOM_ARROW,
                self._force_arrow_width,
                point,
                tip,
            )
            scn.ngeom = 1

    def _apply_force(self):
        # mj_applyFT adds its contribution into qfrc_target, so we must clear the
        # previously applied external force each step before adding a new one.
        self.data.qfrc_applied[:] = 0.0

        step_in_interval = self._env_step_counter % self._ext_force_interval
        if 100 < step_in_interval <= (100 + self._ext_force_duration):
            if step_in_interval == 101:
                self._force_dir = -self._force_dir

            body_id = self.model.body("base").id
            point = self.data.xpos[body_id] + np.array([0.0, 0.0, 0.01])  # applying force slightly above the center of mass
            force = np.array([0.0, self._force_dir * self._ext_force_magn, 0.0])
            torque = np.zeros(3)

            self._draw_force_visualization(point, force)

            mujoco.mj_applyFT(
                self.model,
                self.data,
                force,
                torque,
                point,
                body_id,
                self.data.qfrc_applied
            )

            print(force)
        else:
            self._clear_force_visualization()

    # ==========================================================
    # DEPTH IMAGE
    # ==========================================================

    def get_depth_image(self):
        if not self._depth_enabled:
            return None

        if self._depth_data is None:
            if not self._depth_renderer_lock.acquire(blocking=False):
                return None
            try:
                self._ensure_depth_renderer()
            finally:
                self._depth_renderer_lock.release()

        if not self._sim_data_lock.acquire(blocking=False):
            return None

        try:
            self._depth_data.time = self.data.time
            self._depth_data.qpos[:] = self.data.qpos
            self._depth_data.qvel[:] = self.data.qvel
            if self.model.na > 0:
                self._depth_data.act[:] = self.data.act
            if self.model.nmocap > 0:
                self._depth_data.mocap_pos[:] = self.data.mocap_pos
                self._depth_data.mocap_quat[:] = self.data.mocap_quat
        finally:
            self._sim_data_lock.release()

        if not self._depth_renderer_lock.acquire(blocking=False):
            return None
        try:
            self._ensure_depth_renderer()

            mujoco.mj_forward(self.model, self._depth_data)
            self._depth_renderer.update_scene(self._depth_data, camera=self._depth_camera_name)

            if hasattr(self._depth_renderer, "enable_depth_rendering"):
                self._depth_renderer.enable_depth_rendering()
                try:
                    depth = self._depth_renderer.render()
                finally:
                    self._depth_renderer.disable_depth_rendering()
            else:
                depth = self._depth_renderer.render(depth=True)
        finally:
            self._depth_renderer_lock.release()

        depth_image_m = np.asarray(depth, dtype=np.float32)
        valid_range_mask = np.isfinite(depth_image_m)
        valid_range_mask &= depth_image_m >= self._depth_min_range_m
        valid_range_mask &= depth_image_m <= self._depth_max_range_m

        result = np.zeros_like(depth_image_m, dtype=np.float32)
        result[valid_range_mask] = depth_image_m[valid_range_mask]
        return result
